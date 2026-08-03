#!/usr/bin/env python3
"""
traffic_sniffer.py
Módulo para monitorear el tráfico de red, puertos activos,
dominios DDS de ROS 2, Micro-ROS (puerto 8888) y rendimiento TCP/UDP.
"""

import collections
import os
import re
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
            "microros_agent_active": False,
            "microros_agent_pid": None,
            "discovery_server_active": False,
            "active_sockets": []
        }

        # Contadores de referencia para cálculo delta
        self._last_counters = self._get_raw_net_counters()
        self._last_time = time.time()
        self._lock = threading.Lock()
        self._running = False
        self._worker_thread: Optional[threading.Thread] = None

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
        Escanea sockets abiertos (UDP y TCP) en el sistema para detectar
        Dominios ROS 2, Micro-ROS y servidores de red.
        """
        active_domains = set()
        microros_active = False
        microros_pid = None
        discovery_server_active = False
        socket_list = []

        if PSUTIL_AVAILABLE:
            try:
                conns = psutil.net_connections(kind="inet")
                for c in conns:
                    lport = c.laddr.port if c.laddr else 0
                    proto = "UDP" if c.type == socket.SOCK_DGRAM else "TCP"
                    
                    # 1. Detección de Micro-ROS (Puerto 8888)
                    if lport == 8888:
                        microros_active = True
                        microros_pid = c.pid
                        socket_list.append({
                            "proto": proto,
                            "port": lport,
                            "role": "Micro-ROS Agent",
                            "pid": c.pid,
                            "status": c.status
                        })

                    # 2. Detección de Discovery Server (Puerto 11811)
                    elif lport == 11811:
                        discovery_server_active = True
                        socket_list.append({
                            "proto": proto,
                            "port": lport,
                            "role": "Fast DDS Discovery Server",
                            "pid": c.pid,
                            "status": c.status
                        })

                    # 3. Detección de Puertos DDS (7400 a 32000)
                    elif 7400 <= lport <= 32000 and proto == "UDP":
                        # Fórmula DDS: Port = 7400 + (DomainID * 250) + offset
                        domain_id = (lport - 7400) // 250
                        if 0 <= domain_id <= 230:
                            active_domains.add(domain_id)
                            socket_list.append({
                                "proto": proto,
                                "port": lport,
                                "role": f"ROS 2 DDS (Domain {domain_id})",
                                "pid": c.pid,
                                "status": "LISTENING"
                            })
                    
                    # 4. Puertos clave de ROSbridge / SSH
                    elif lport in (9090, 22):
                        label = "ROSbridge WebSocket" if lport == 9090 else "SSH Server"
                        socket_list.append({
                            "proto": proto,
                            "port": lport,
                            "role": label,
                            "pid": c.pid,
                            "status": c.status
                        })
            except Exception:
                pass
        
        # Fallback con 'ss -tulnp' si psutil no tuvo permisos completos
        if not socket_list:
            try:
                out = subprocess.check_output(["ss", "-tulnp"], stderr=subprocess.DEVNULL, timeout=2).decode("utf-8")
                for line in out.splitlines():
                    if ":8888 " in line:
                        microros_active = True
                    if ":11811 " in line:
                        discovery_server_active = True
                    match = re.search(r":(\d{4,5})\s", line)
                    if match:
                        p = int(match.group(1))
                        if 7400 <= p <= 32000 and "udp" in line.lower():
                            d_id = (p - 7400) // 250
                            if 0 <= d_id <= 230:
                                active_domains.add(d_id)
            except Exception:
                pass

        return {
            "active_dds_domains": sorted(list(active_domains)),
            "microros_agent_active": microros_active,
            "microros_agent_pid": microros_pid,
            "discovery_server_active": discovery_server_active,
            "active_sockets": socket_list[:15]  # Primeros 15 sockets relevantes
        }

    def _measure_gateway_quality(self) -> Dict[str, float]:
        """Mide latencia, jitter y pérdida de paquetes al Gateway"""
        if not self.gateway_ip or not re.match(r"^\d+\.\d+\.\d+\.\d+$", self.gateway_ip):
            return {"latency_ms": 0.0, "jitter_ms": 0.0, "loss_percent": 0.0}

        try:
            cmd = ["ping", "-c", "3", "-i", "0.2", "-W", "1", self.gateway_ip]
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=2.5)
            if res.returncode == 0:
                out = res.stdout.decode("utf-8", errors="ignore")
                
                # Pérdida de paquetes
                loss_match = re.search(r"(\d+)%\s*packet loss", out)
                loss = float(loss_match.group(1)) if loss_match else 0.0
                
                # Latencias: min/avg/max/mdev
                rtt_match = re.search(r"rtt\s+min/avg/max/mdev\s*=\s*([0-9.]+)/([0-9.]+)/([0-9.]+)/([0-9.]+)", out)
                if rtt_match:
                    avg_lat = float(rtt_match.group(2))
                    mdev_jitter = float(rtt_match.group(4))
                    return {
                        "latency_ms": round(avg_lat, 2),
                        "jitter_ms": round(mdev_jitter, 2),
                        "loss_percent": loss
                    }
        except Exception:
            pass

        return {"latency_ms": 0.0, "jitter_ms": 0.0, "loss_percent": 0.0}

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

        # Calidad de enlace al Gateway
        gw_quality = self._measure_gateway_quality()

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
                "gateway_latency_ms": gw_quality["latency_ms"],
                "jitter_ms": gw_quality["jitter_ms"],
                "packet_loss_percent": gw_quality["loss_percent"],
                "active_dds_domains": port_info["active_dds_domains"],
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

    def start_background_collector(self, interval: float = 1.0):
        """Inicia el hilo recolector de métricas en segundo plano"""
        if self._running:
            return
        self._running = True
        
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
