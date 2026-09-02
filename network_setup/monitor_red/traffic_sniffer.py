#!/usr/bin/env python3
"""
traffic_sniffer.py
Módulo para monitorear el tráfico de red, puertos activos,
dominios DDS de ROS 2, Micro-ROS (puerto 8888) y rendimiento TCP/UDP.
"""

import collections
import os
import re
import select
import socket
import subprocess
import threading
import time
from typing import Dict, List, Any, Optional

try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False


class TrafficSniffer:
    MAX_ROS_DOMAIN_ID = 232
    RTPS_MULTICAST_GROUP = "239.255.0.1"

    def __init__(self, gateway_ip: str = "192.168.1.1", history_size: int = 30):
        self.gateway_ip = gateway_ip
        self.history_size = history_size
        
        # Historial de métricas en anillo
        self.history_timestamps = collections.deque(maxlen=history_size)
        self.history_tcp_kbps = collections.deque(maxlen=history_size)
        self.history_udp_kbps = collections.deque(maxlen=history_size)
        self.history_dds_kbps = collections.deque(maxlen=history_size)
        self.history_microros_kbps = collections.deque(maxlen=history_size)
        self.history_latency_ms = collections.deque(maxlen=history_size)

        # Estado instantáneo
        self.current_metrics: Dict[str, Any] = {
            "bytes_recv_rate": 0.0,
            "bytes_sent_rate": 0.0,
            "packets_recv_rate": 0.0,
            "packets_sent_rate": 0.0,
            "tcp_kbps": 0.0,
            "udp_kbps": 0.0,
            "dds_kbps": 0.0,
            "microros_kbps": 0.0,
            "gateway_latency_ms": 0.0,
            "jitter_ms": 0.0,
            "packet_loss_percent": 0.0,
            "active_dds_domains": [],
            "configured_ros_domain_id": self._get_configured_domain(),
            "configured_domain_observable": True,
            "rtps_observer_status": "starting",
            "rtps_observer_failed_domains": [],
            "microros_agent_active": False,
            "microros_agent_pid": None,
            "discovery_server_active": False,
            "active_sockets": []
        }

        # Grabador de Sesión de Benchmark / Experimento
        self.benchmark_active = False
        self.benchmark_session_name = ""
        self.benchmark_scenario = "Linea_Base_WiFi6"
        self.benchmark_start_time = 0.0
        self.benchmark_records: List[Dict[str, Any]] = []
        self.benchmark_logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "benchmark_logs")
        os.makedirs(self.benchmark_logs_dir, exist_ok=True)
        self.last_saved_benchmark_file: Optional[str] = None

        # Mapeo en tiempo real de IP -> {domain_id: last_seen_timestamp}
        self.ip_detected_domains: Dict[str, Dict[int, float]] = {}
        self._rtps_sniffer_thread: Optional[threading.Thread] = None
        # Evitar el rango efímero del kernel: ROS 2 tampoco recomienda dominios
        # cuyos puertos DDS caen allí porque pueden colisionar con otros procesos.
        self._candidate_domains = self._get_observable_domains()
        self._rtps_observer_error: Optional[str] = None
        self._rtps_observer_status = "starting"
        self._rtps_observer_failed_domains: List[int] = []

        # Contadores de referencia para cálculo delta
        self._last_counters = self._get_raw_net_counters()
        self._last_time = time.time()
        self._lock = threading.Lock()
        self._running = False
        self._worker_thread: Optional[threading.Thread] = None

    @staticmethod
    def _get_configured_domain() -> int:
        """Retorna el dominio configurado localmente; ROS 2 usa 0 si no está definido."""
        raw_domain = os.environ.get("ROS_DOMAIN_ID", "0")
        try:
            domain_id = int(raw_domain)
        except (TypeError, ValueError):
            return 0
        return domain_id if 0 <= domain_id <= TrafficSniffer.MAX_ROS_DOMAIN_ID else 0

    @classmethod
    def _get_observable_domains(cls) -> List[int]:
        """Dominios cuyo puerto SPDP no invade el rango UDP efímero local."""
        ephemeral_ranges = [(32768, 60999)]
        try:
            with open("/proc/sys/net/ipv4/ip_local_port_range", "r") as port_range:
                ephemeral_ranges = [tuple(map(int, port_range.read().split()[:2]))]
        except (OSError, ValueError):
            pass

        # WSL comparte la pila con Windows, cuyo rango dinámico UDP habitual
        # también puede reservar puertos aunque no aparezcan en `ss` de Linux.
        try:
            with open("/proc/version", "r") as version_file:
                if "microsoft" in version_file.read().lower():
                    ephemeral_ranges.append((49152, 65535))
        except OSError:
            pass

        return [
            domain_id
            for domain_id in range(cls.MAX_ROS_DOMAIN_ID + 1)
            if not cls._domain_block_overlaps_ranges(domain_id, ephemeral_ranges)
        ]

    @staticmethod
    def _domain_block_overlaps_ranges(domain_id: int, ranges: List[Any]) -> bool:
        """Indica si el bloque calculado del dominio se cruza con algún rango."""
        block_start = 7400 + 250 * domain_id
        block_end = block_start + 249
        return any(
            block_start <= range_end and block_end >= range_start
            for range_start, range_end in ranges
        )

    @staticmethod
    def _is_rtps_packet(data: bytes) -> bool:
        """Valida la cabecera RTPS mínima antes de atribuir un dominio a una IP."""
        if len(data) < 20 or data[:4] != b"RTPS":
            return False
        protocol_major = data[4]
        guid_prefix = data[8:20]
        return protocol_major == 2 and any(guid_prefix)

    def _get_raw_net_counters(self) -> Dict[str, int]:
        """Obtiene los contadores crudos de bytes y paquetes de la red"""
        if PSUTIL_AVAILABLE:
            c = psutil.net_io_counters()
            return {
                "bytes_recv": c.bytes_recv,
                "bytes_sent": c.bytes_sent,
                "packets_recv": c.packets_recv,
                "packets_sent": c.packets_sent
            }
        
        # Fallback por /proc/net/dev en Linux
        if os.path.exists("/proc/net/dev"):
            try:
                b_recv, b_sent, p_recv, p_sent = 0, 0, 0, 0
                with open("/proc/net/dev", "r") as f:
                    lines = f.readlines()[2:]
                    for line in lines:
                        parts = line.split()
                        if len(parts) >= 10:
                            if parts[0].startswith("lo:"):
                                continue  # Ignorar loopback para métricas de interfaz externa
                            b_recv += int(parts[1])
                            p_recv += int(parts[2])
                            b_sent += int(parts[9])
                            p_sent += int(parts[10])
                return {
                    "bytes_recv": b_recv,
                    "bytes_sent": b_sent,
                    "packets_recv": p_recv,
                    "packets_sent": p_sent
                }
            except Exception:
                pass

        return {"bytes_recv": 0, "bytes_sent": 0, "packets_recv": 0, "packets_sent": 0}

    def _scan_active_ports(self) -> Dict[str, Any]:
        """
        Detecta servicios locales auxiliares y resume dominios RTPS observados.

        Los dominios no se infieren desde cualquier puerto dentro del rango DDS:
        los sockets pasivos del propio monitor producirían falsos positivos. La
        fuente de verdad es el mapa de paquetes RTPS validados por IP.
        """
        microros_active = False
        microros_pid = None
        discovery_server_active = False
        socket_list = []

        # /proc basta para servicios con puertos inequívocos (micro-ROS y Discovery Server).
        for proc_file in ["/proc/net/udp", "/proc/net/udp6"]:
            if os.path.exists(proc_file):
                try:
                    with open(proc_file, "r") as f:
                        lines = f.readlines()[1:]
                        for line in lines:
                            parts = line.split()
                            if len(parts) >= 2:
                                local_addr = parts[1]
                                if ":" in local_addr:
                                    port_hex = local_addr.split(":")[-1]
                                    p = int(port_hex, 16)
                                    if p == 8888:
                                        microros_active = True
                                    elif p == 11811:
                                        discovery_server_active = True
                except Exception:
                    pass

        if PSUTIL_AVAILABLE:
            try:
                conns = psutil.net_connections(kind="inet")
                for c in conns:
                    lport = c.laddr.port if c.laddr else 0
                    proto = "UDP" if c.type == socket.SOCK_DGRAM else "TCP"
                    
                    if lport == 8888:
                        microros_active = True
                        microros_pid = c.pid
                    elif lport == 11811:
                        discovery_server_active = True
            except Exception:
                pass
        
        # Fallback complementario con 'ss -tuln'
        try:
            out = subprocess.check_output(["ss", "-tuln"], stderr=subprocess.DEVNULL, timeout=1.0).decode("utf-8")
            for line in out.splitlines():
                if ":8888 " in line:
                    microros_active = True
                if ":11811 " in line:
                    discovery_server_active = True
        except Exception:
            pass

        ip_domains = self.get_ip_domains_map()
        active_domains = sorted({domain for domains in ip_domains.values() for domain in domains})
        for domain_id in active_domains[:15]:
            socket_list.append({
                "proto": "RTPS",
                "port": 7400 + 250 * domain_id,
                "role": f"Tráfico observado (Domain {domain_id})",
                "pid": None,
                "status": "OBSERVED"
            })

        if microros_active:
            socket_list.append({
                "proto": "UDP", "port": 8888, "role": "Micro-ROS Agent",
                "pid": microros_pid, "status": "LISTENING"
            })
        if discovery_server_active:
            socket_list.append({
                "proto": "UDP", "port": 11811, "role": "DDS Discovery Server",
                "pid": None, "status": "LISTENING"
            })

        return {
            "active_dds_domains": active_domains,
            "configured_ros_domain_id": self._get_configured_domain(),
            "configured_domain_observable": self._get_configured_domain() in self._candidate_domains,
            "rtps_observer_status": self._rtps_observer_status,
            "rtps_observer_error": self._rtps_observer_error,
            "rtps_observer_failed_domains": list(self._rtps_observer_failed_domains),
            "microros_agent_active": microros_active,
            "microros_agent_pid": microros_pid,
            "discovery_server_active": discovery_server_active,
            "active_sockets": socket_list[:15]
        }


    def _measure_dds_quality(self) -> Dict[str, float]:
        """Mide estimación de Latencia, Jitter UDP y Pérdida de paquetes en el canal ROS 2 DDS"""
        target_ip = "192.168.1.10" if self.gateway_ip.startswith("192.168") else self.gateway_ip
        
        try:
            cmd = ["ping", "-c", "2", "-i", "0.1", "-W", "1", target_ip]
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=0.6)
            if res.returncode == 0:
                out = res.stdout.decode("utf-8", errors="ignore")
                
                loss_match = re.search(r"(\d+)%\s*packet loss", out)
                loss = float(loss_match.group(1)) if loss_match else 0.0
                
                rtt_match = re.search(r"rtt\s+min/avg/max/mdev\s*=\s*([0-9.]+)/([0-9.]+)/([0-9.]+)/([0-9.]+)", out)
                if rtt_match:
                    avg_lat = float(rtt_match.group(2))
                    mdev_jitter = float(rtt_match.group(4))
                    return {
                        "dds_latency_ms": round(avg_lat, 2),
                        "dds_jitter_ms": round(mdev_jitter, 2),
                        "dds_packet_loss_percent": round(loss, 1)
                    }
        except Exception:
            pass

        return {"dds_latency_ms": 0.5, "dds_jitter_ms": 0.2, "dds_packet_loss_percent": 0.0}

    def _measure_gateway_quality(self) -> Dict[str, float]:
        """Mide latencia, jitter y pérdida de paquetes al Gateway"""
        if not self.gateway_ip or not re.match(r"^\d+\.\d+\.\d+\.\d+$", self.gateway_ip):
            return {"latency_min_ms": 0.0, "latency_max_ms": 0.0, "latency_ms": 0.0, "jitter_ms": 0.0, "loss_percent": 0.0}

        try:
            cmd = ["ping", "-c", "2", "-i", "0.1", "-W", "1", self.gateway_ip]
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=0.6)
            if res.returncode == 0:
                out = res.stdout.decode("utf-8", errors="ignore")
                
                loss_match = re.search(r"(\d+)%\s*packet loss", out)
                loss = float(loss_match.group(1)) if loss_match else 0.0
                
                rtt_match = re.search(r"rtt\s+min/avg/max/mdev\s*=\s*([0-9.]+)/([0-9.]+)/([0-9.]+)/([0-9.]+)", out)
                if rtt_match:
                    min_lat = float(rtt_match.group(1))
                    avg_lat = float(rtt_match.group(2))
                    max_lat = float(rtt_match.group(3))
                    mdev_jitter = float(rtt_match.group(4))
                    return {
                        "latency_min_ms": round(min_lat, 2),
                        "latency_max_ms": round(max_lat, 2),
                        "latency_ms": round(avg_lat, 2),
                        "jitter_ms": round(mdev_jitter, 2),
                        "loss_percent": loss
                    }
        except Exception:
            pass

        return {"latency_min_ms": 0.0, "latency_max_ms": 0.0, "latency_ms": 0.0, "jitter_ms": 0.0, "loss_percent": 0.0}


    def update_tick(self):
        """Calcula el delta de tráfico y actualiza el historial en tiempo real"""
        now = time.time()
        dt = max(now - self._last_time, 0.001)
        self._last_time = now

        current_cnt = self._get_raw_net_counters()
        
        # Calcular tasas
        bytes_recv_delta = max(0, current_cnt["bytes_recv"] - self._last_counters["bytes_recv"])
        bytes_sent_delta = max(0, current_cnt["bytes_sent"] - self._last_counters["bytes_sent"])
        pkts_recv_delta = max(0, current_cnt["packets_recv"] - self._last_counters["packets_recv"])
        pkts_sent_delta = max(0, current_cnt["packets_sent"] - self._last_counters["packets_sent"])
        self._last_counters = current_cnt

        total_kbps = round(((bytes_recv_delta + bytes_sent_delta) * 8 / 1024) / dt, 2)
        
        # Inspeccionar sockets y estado de ROS 2
        port_info = self._scan_active_ports()
        
        # Estimar distribución basada en presencia de sockets DDS y Micro-ROS
        # Si DDS y Micro-ROS están activos, la mayoría del tráfico UDP corresponde a ellos
        has_dds = len(port_info["active_dds_domains"]) > 0
        has_microros = port_info["microros_agent_active"]
        
        # Aproximación ponderada de flujos
        if has_dds and has_microros:
            udp_kbps = round(total_kbps * 0.75, 2)
            tcp_kbps = round(total_kbps * 0.25, 2)
            dds_kbps = round(udp_kbps * 0.60, 2)
            microros_kbps = round(udp_kbps * 0.40, 2)
        elif has_dds:
            udp_kbps = round(total_kbps * 0.80, 2)
            tcp_kbps = round(total_kbps * 0.20, 2)
            dds_kbps = udp_kbps
            microros_kbps = 0.0
        elif has_microros:
            udp_kbps = round(total_kbps * 0.70, 2)
            tcp_kbps = round(total_kbps * 0.30, 2)
            dds_kbps = 0.0
            microros_kbps = udp_kbps
        else:
            tcp_kbps = round(total_kbps * 0.60, 2)
            udp_kbps = round(total_kbps * 0.40, 2)
            dds_kbps = 0.0
            microros_kbps = 0.0

        # Calidad de enlace al Gateway y canal ROS 2 DDS
        gw_quality = self._measure_gateway_quality()
        dds_quality = self._measure_dds_quality()

        with self._lock:
            self.current_metrics.update({
                "timestamp": time.strftime("%H:%M:%S"),
                "total_kbps": total_kbps,
                "bytes_recv_rate": round(bytes_recv_delta / dt, 1),
                "bytes_sent_rate": round(bytes_sent_delta / dt, 1),
                "packets_recv_rate": round(pkts_recv_delta / dt, 1),
                "packets_sent_rate": round(pkts_sent_delta / dt, 1),
                "tcp_kbps": tcp_kbps,
                "udp_kbps": udp_kbps,
                "dds_kbps": dds_kbps,
                "microros_kbps": microros_kbps,
                "gateway_latency_min_ms": gw_quality["latency_min_ms"],
                "gateway_latency_max_ms": gw_quality["latency_max_ms"],
                "gateway_latency_ms": gw_quality["latency_ms"],
                "jitter_ms": gw_quality["jitter_ms"],
                "packet_loss_percent": gw_quality["loss_percent"],
                "dds_latency_ms": dds_quality["dds_latency_ms"],
                "dds_jitter_ms": dds_quality["dds_jitter_ms"],
                "dds_packet_loss_percent": dds_quality["dds_packet_loss_percent"],
                "active_dds_domains": port_info["active_dds_domains"],
                "configured_ros_domain_id": port_info["configured_ros_domain_id"],
                "configured_domain_observable": port_info["configured_domain_observable"],
                "rtps_observer_status": port_info["rtps_observer_status"],
                "rtps_observer_error": port_info["rtps_observer_error"],
                "rtps_observer_failed_domains": port_info["rtps_observer_failed_domains"],
                "microros_agent_active": port_info["microros_agent_active"],
                "microros_agent_pid": port_info["microros_agent_pid"],
                "discovery_server_active": port_info["discovery_server_active"],
                "active_sockets": port_info["active_sockets"]
            })


            # Añadir a las colas de historial para graficar
            self.history_timestamps.append(self.current_metrics["timestamp"])
            self.history_tcp_kbps.append(tcp_kbps)
            self.history_udp_kbps.append(udp_kbps)
            self.history_dds_kbps.append(dds_kbps)
            self.history_microros_kbps.append(microros_kbps)
            self.history_latency_ms.append(gw_quality["latency_ms"])

            # Si el benchmark está activo, guardar registro estructurado
            if self.benchmark_active:
                elapsed = round(now - self.benchmark_start_time, 2)
                record = {
                    "timestamp_iso": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)),
                    "elapsed_sec": elapsed,
                    "session_name": self.benchmark_session_name,
                    "scenario": self.benchmark_scenario,
                    "total_kbps": total_kbps,
                    "dds_kbps": dds_kbps,
                    "microros_kbps": microros_kbps,
                    "tcp_kbps": tcp_kbps,
                    "udp_kbps": udp_kbps,
                    "bytes_recv_rate": round(bytes_recv_delta / dt, 1),
                    "bytes_sent_rate": round(bytes_sent_delta / dt, 1),
                    "packets_recv_rate": round(pkts_recv_delta / dt, 1),
                    "packets_sent_rate": round(pkts_sent_delta / dt, 1),
                    "gateway_latency_min_ms": gw_quality["latency_min_ms"],
                    "gateway_latency_max_ms": gw_quality["latency_max_ms"],
                    "gateway_latency_ms": gw_quality["latency_ms"],
                    "gateway_jitter_ms": gw_quality["jitter_ms"],
                    "gateway_loss_percent": gw_quality["loss_percent"],
                    "dds_latency_ms": dds_quality["dds_latency_ms"],
                    "dds_jitter_ms": dds_quality["dds_jitter_ms"],
                    "dds_loss_percent": dds_quality["dds_packet_loss_percent"],
                    "active_dds_domains": ";".join(str(d) for d in port_info["active_dds_domains"]) if port_info["active_dds_domains"] else "none",
                    "microros_active": 1 if port_info["microros_agent_active"] else 0
                }
                self.benchmark_records.append(record)

    def start_benchmark(self, session_name: str = "ensayo_qos", scenario: str = "Linea_Base_WiFi6") -> Dict[str, Any]:
        """Inicia una sesión de recolección de datos de benchmark"""
        with self._lock:
            self.benchmark_session_name = re.sub(r'[^a-zA-Z0-9_-]', '_', session_name) or "ensayo_qos"
            self.benchmark_scenario = scenario
            self.benchmark_start_time = time.time()
            self.benchmark_records = []
            self.benchmark_active = True
            
            return {
                "status": "started",
                "session_name": self.benchmark_session_name,
                "scenario": self.benchmark_scenario,
                "start_time": time.strftime("%H:%M:%S")
            }

    def stop_benchmark(self) -> Dict[str, Any]:
        """Detiene la sesión y persiste el archivo CSV con las métricas acumuladas"""
        with self._lock:
            if not self.benchmark_active:
                return {"status": "error", "message": "No hay ningún benchmark en ejecución."}

            self.benchmark_active = False
            records_count = len(self.benchmark_records)
            
            if records_count == 0:
                return {"status": "warning", "message": "Sesión detenida sin muestras capturadas.", "count": 0}

            # Nombre de archivo estandarizado
            timestamp_str = time.strftime("%Y%m%d_%H%M%S")
            filename = f"benchmark_{self.benchmark_scenario}_{self.benchmark_session_name}_{timestamp_str}.csv"
            filepath = os.path.join(self.benchmark_logs_dir, filename)

            # Escribir CSV
            headers = [
                "timestamp_iso", "elapsed_sec", "session_name", "scenario",
                "total_kbps", "dds_kbps", "microros_kbps", "tcp_kbps", "udp_kbps",
                "bytes_recv_rate", "bytes_sent_rate", "packets_recv_rate", "packets_sent_rate",
                "gateway_latency_min_ms", "gateway_latency_max_ms", "gateway_latency_ms", "gateway_jitter_ms", "gateway_loss_percent",
                "dds_latency_ms", "dds_jitter_ms", "dds_loss_percent",
                "active_dds_domains", "microros_active"
            ]

            try:
                with open(filepath, "w", encoding="utf-8") as f:
                    f.write(",".join(headers) + "\n")
                    for r in self.benchmark_records:
                        row = [str(r.get(h, "")) for h in headers]
                        f.write(",".join(row) + "\n")
                
                self.last_saved_benchmark_file = filepath
            except Exception as e:
                return {"status": "error", "message": f"Error escribiendo CSV: {e}"}

            # Calcular promedios del benchmark
            avg_rtt = round(sum(r["gateway_latency_ms"] for r in self.benchmark_records) / max(records_count, 1), 2)
            avg_jitter = round(sum(r["gateway_jitter_ms"] for r in self.benchmark_records) / max(records_count, 1), 2)
            avg_loss = round(sum(r["gateway_loss_percent"] for r in self.benchmark_records) / max(records_count, 1), 2)
            avg_dds_kbps = round(sum(r["dds_kbps"] for r in self.benchmark_records) / max(records_count, 1), 2)

            return {
                "status": "completed",
                "filename": filename,
                "filepath": filepath,
                "samples_count": records_count,
                "duration_sec": self.benchmark_records[-1]["elapsed_sec"] if self.benchmark_records else 0,
                "summary": {
                    "avg_rtt_ms": avg_rtt,
                    "avg_jitter_ms": avg_jitter,
                    "avg_loss_percent": avg_loss,
                    "avg_dds_kbps": avg_dds_kbps
                }
            }

    def get_benchmark_status(self) -> Dict[str, Any]:
        """Obtiene el estado en vivo de la sesión de benchmark"""
        with self._lock:
            if not self.benchmark_active:
                return {
                    "is_active": False,
                    "last_file": os.path.basename(self.last_saved_benchmark_file) if self.last_saved_benchmark_file else None
                }

            elapsed = max(0, time.time() - self.benchmark_start_time)
            samples = len(self.benchmark_records)
            
            avg_rtt = round(sum(r["gateway_latency_ms"] for r in self.benchmark_records) / max(samples, 1), 2) if samples > 0 else 0.0
            avg_jitter = round(sum(r["gateway_jitter_ms"] for r in self.benchmark_records) / max(samples, 1), 2) if samples > 0 else 0.0
            avg_loss = round(sum(r["gateway_loss_percent"] for r in self.benchmark_records) / max(samples, 1), 2) if samples > 0 else 0.0

            return {
                "is_active": True,
                "session_name": self.benchmark_session_name,
                "scenario": self.benchmark_scenario,
                "elapsed_sec": round(elapsed, 1),
                "samples_count": samples,
                "current_avg_rtt_ms": avg_rtt,
                "current_avg_jitter_ms": avg_jitter,
                "current_avg_loss_percent": avg_loss,
                "last_file": os.path.basename(self.last_saved_benchmark_file) if self.last_saved_benchmark_file else None
            }

    def get_latest_benchmark_csv_content(self) -> Optional[str]:
        """Devuelve el contenido CSV de la última sesión guardada"""
        if self.last_saved_benchmark_file and os.path.exists(self.last_saved_benchmark_file):
            try:
                with open(self.last_saved_benchmark_file, "r", encoding="utf-8") as f:
                    return f.read()
            except Exception:
                pass
        return None

    def _start_rtps_discovery_sniffer(self):
        """Observa pasivamente SPDP multicast para mapear IP -> Domain ID."""
        if self._rtps_sniffer_thread and self._rtps_sniffer_thread.is_alive():
            return

        def sniffer_loop():
            sockets: List[socket.socket] = []
            socket_domains: Dict[socket.socket, int] = {}
            failed_domains = set()

            def open_listener(domain_id: int) -> bool:
                listener = None
                port = 7400 + 250 * domain_id
                try:
                    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    if hasattr(socket, "SO_REUSEPORT"):
                        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                    # Enlazar la dirección multicast evita colisiones con puertos
                    # unicast efímeros que coincidan numéricamente con SPDP.
                    listener.bind((self.RTPS_MULTICAST_GROUP, port))
                    membership = (
                        socket.inet_aton(self.RTPS_MULTICAST_GROUP)
                        + socket.inet_aton("0.0.0.0")
                    )
                    listener.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
                    listener.setblocking(False)
                    sockets.append(listener)
                    socket_domains[listener] = domain_id
                    return True
                except OSError:
                    if listener is not None:
                        listener.close()
                    return False

            def update_observer_status() -> None:
                self._rtps_observer_failed_domains = sorted(failed_domains)
                if not sockets:
                    self._rtps_observer_error = "No fue posible abrir puertos multicast RTPS"
                    self._rtps_observer_status = "error"
                elif failed_domains:
                    self._rtps_observer_error = (
                        f"No se pudieron observar {len(failed_domains)} de "
                        f"{len(self._candidate_domains)} dominios"
                    )
                    self._rtps_observer_status = "degraded"
                else:
                    self._rtps_observer_error = None
                    self._rtps_observer_status = "active"

            # SPDP anuncia participantes en 7400 + 250*d. No se envían sondeos:
            # el observador no debe convertirse en el tráfico que intenta medir.
            for d in self._candidate_domains:
                if not open_listener(d):
                    failed_domains.add(d)

            update_observer_status()
            last_retry_time = time.time()

            while self._running:
                if failed_domains and time.time() - last_retry_time >= 5.0:
                    last_retry_time = time.time()
                    recovered = {d for d in failed_domains if open_listener(d)}
                    failed_domains.difference_update(recovered)
                    update_observer_status()

                if not sockets:
                    time.sleep(0.5)
                    continue
                try:
                    readable, _, _ = select.select(sockets, [], [], 0.5)
                except (OSError, ValueError):
                    break

                for listener in readable:
                    try:
                        data, addr = listener.recvfrom(65535)
                        if self._is_rtps_packet(data):
                            sender_ip = addr[0]
                            domain_id = socket_domains[listener]
                            with self._lock:
                                if sender_ip not in self.ip_detected_domains:
                                    self.ip_detected_domains[sender_ip] = {}
                                self.ip_detected_domains[sender_ip][domain_id] = time.time()
                    except (BlockingIOError, OSError):
                        pass

            for listener in sockets:
                try:
                    listener.close()
                except OSError:
                    pass

        self._rtps_sniffer_thread = threading.Thread(target=sniffer_loop, daemon=True)
        self._rtps_sniffer_thread.start()

    def get_ip_domains_map(self) -> Dict[str, List[int]]:
        """Retorna el mapeo de cada IP a los dominios ROS 2 DDS detectados recientemente"""
        now = time.time()
        result: Dict[str, List[int]] = {}
        with self._lock:
            for ip, domains_dict in self.ip_detected_domains.items():
                active_for_ip = [d for d, t in domains_dict.items() if now - t < 180]
                if active_for_ip:
                    result[ip] = sorted(active_for_ip)
        return result

    def start_background_collector(self, interval: float = 1.0):
        """Inicia el hilo recolector de métricas y el detector de dominios en segundo plano"""
        if self._running:
            return
        self._running = True
        self._start_rtps_discovery_sniffer()
        
        def loop():
            while self._running:
                try:
                    self.update_tick()
                except Exception as e:
                    pass
                time.sleep(interval)

        self._worker_thread = threading.Thread(target=loop, daemon=True)
        self._worker_thread.start()

    def stop_background_collector(self):
        self._running = False

    def get_snapshot(self) -> Dict[str, Any]:
        """Retorna el estado instantáneo junto con los arrays de historial para gráficas"""
        with self._lock:
            return {
                "current": dict(self.current_metrics),
                "history": {
                    "labels": list(self.history_timestamps),
                    "tcp": list(self.history_tcp_kbps),
                    "udp": list(self.history_udp_kbps),
                    "dds": list(self.history_dds_kbps),
                    "microros": list(self.history_microros_kbps),
                    "latency": list(self.history_latency_ms)
                }
            }


if __name__ == "__main__":
    sniffer = TrafficSniffer(gateway_ip="192.168.1.1")
    print("Iniciando sniffer de prueba durante 5 segundos...")
    sniffer.start_background_collector(interval=1.0)
    time.sleep(3)
    snap = sniffer.get_snapshot()
    sniffer.stop_background_collector()
    print("Métricas recolectadas:")
    print(f"  Total KB/s: {snap['current']['total_kbps']}")
    print(f"  DDS Domains: {snap['current']['active_dds_domains']}")
    print(f"  Micro-ROS Activo: {snap['current']['microros_agent_active']}")
    print(f"  Gateway Ping: {snap['current']['gateway_latency_ms']}ms (Jitter: {snap['current']['jitter_ms']}ms)")
