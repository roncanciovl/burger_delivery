#!/usr/bin/env python3
"""
device_scanner.py
Módulo para escanear y descubrir dispositivos en la subred local,
consultando la tabla ARP/Neighbor del kernel y realizando sondeos de latencia.
"""

import os
import re
import socket
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Optional, Any

# Prefijos MAC conocidos (OUI) para identificación rápida
KNOWN_OUIS = {
    # Espressif (ESP32 / ESP8266)
    "24:6f:28": "Espressif (ESP32)", "30:ae:a4": "Espressif (ESP32)", "a4:cf:12": "Espressif (ESP32)",
    "ac:67:b2": "Espressif (ESP32)", "ec:62:60": "Espressif (ESP32)", "48:55:19": "Espressif (ESP32)",
    "c8:f0:9e": "Espressif (ESP32)", "84:f7:03": "Espressif (ESP32)", "24:0a:c4": "Espressif (ESP32)",
    "3c:71:bf": "Espressif (ESP32)", "70:b8:f6": "Espressif (ESP32)", "10:52:1c": "Espressif (ESP32)",
    "34:85:18": "Espressif (ESP32)", "40:22:d8": "Espressif (ESP32)", "7c:df:a1": "Espressif (ESP32)",
    "80:64:6f": "Espressif (ESP32)", "94:b9:7e": "Espressif (ESP32)", "94:e6:86": "Espressif (ESP32)",
    "a4:e5:7c": "Espressif (ESP32)", "b4:e6:2d": "Espressif (ESP32)", "bc:dd:c2": "Espressif (ESP32)",
    "c4:4f:33": "Espressif (ESP32)", "d8:a0:1d": "Espressif (ESP32)", "e0:e2:e6": "Espressif (ESP32)",
    "e8:68:e7": "Espressif (ESP32)", "ec:fa:bc": "Espressif (ESP32)",
    # Raspberry Pi Foundation
    "b8:27:eb": "Raspberry Pi", "dc:a6:32": "Raspberry Pi", "e4:5f:01": "Raspberry Pi",
    "d8:3a:dd": "Raspberry Pi 4/5", "28:cd:c1": "Raspberry Pi",
    # TP-Link
    "50:d4:f7": "TP-Link", "98:25:4a": "TP-Link", "ec:17:2f": "TP-Link",
    "14:eb:b6": "TP-Link", "30:de:4b": "TP-Link", "54:af:97": "TP-Link",
    "70:4f:57": "TP-Link", "84:16:f9": "TP-Link", "b0:4e:26": "TP-Link",
    "c0:06:c3": "TP-Link", "c0:25:e9": "TP-Link", "c4:71:54": "TP-Link",
    "d8:07:b6": "TP-Link", "e8:48:b8": "TP-Link", "f4:f2:6d": "TP-Link"
}

class DeviceScanner:
    def __init__(self, subnet: Optional[str] = None):
        self.gateway_ip = self._get_default_gateway()
        self.local_ip = self._get_local_ip()
        self.subnet = subnet or self._detect_subnet()
        self.last_scan_time = time.time()
        self.cached_devices: List[Dict[str, Any]] = [
            {
                "ip": self.gateway_ip,
                "mac": "D0:78:80:95:13:D1",
                "hostname": "Router",
                "vendor": "TP-Link AX12 (Gateway)",
                "role": "router",
                "label": "Router WiFi (TP-Link AX12 / Gateway)",
                "icon": "router",
                "is_dds_active": False,
                "dds_protocol": "Infraestructura Red",
                "latency_ms": 1.0,
                "status": "online",
                "last_seen": time.strftime("%H:%M:%S")
            },
            {
                "ip": self.local_ip,
                "mac": "LOCAL",
                "hostname": "PC Host",
                "vendor": "Estación de Control",
                "role": "host",
                "label": "PC Principal (Host / Control ROS 2)",
                "icon": "desktop",
                "is_dds_active": True,
                "dds_protocol": "ROS 2 DDS (Domain 42) & Micro-ROS Agent",
                "latency_ms": 0.1,
                "status": "online",
                "last_seen": time.strftime("%H:%M:%S")
            }
        ]
        self.target_domain = int(os.environ.get("ROS_DOMAIN_ID", 42))

    def set_target_domain(self, domain_id: int):
        """Actualiza el ROS_DOMAIN_ID activo"""
        self.target_domain = domain_id
        os.environ["ROS_DOMAIN_ID"] = str(domain_id)


    def _get_default_gateway(self) -> str:
        """Obtiene la IP del Gateway del Router TP-Link AX12 (192.168.1.1)"""
        try:
            cmd = ["ip", "route"]
            output = subprocess.check_output(cmd, stderr=subprocess.DEVNULL, timeout=1.0).decode("utf-8")
            for line in output.splitlines():
                if "192.168.1" in line and "via" in line:
                    parts = line.split()
                    idx = parts.index("via")
                    if idx + 1 < len(parts):
                        return parts[idx + 1]
        except Exception:
            pass
        return "192.168.1.1"

    def _get_local_ip(self) -> str:
        """Obtiene la IP fija asignada en la red del robot (192.168.1.100)"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("192.168.1.1", 80))
            ip = s.getsockname()[0]
            s.close()
            if ip.startswith("192.168."):
                return ip
        except Exception:
            pass
        return "192.168.1.100"

    def _detect_subnet(self) -> str:
        """Prefijo de subred clase C oficial del proyecto ROS 2 (192.168.1)"""
        return "192.168.1"


    def _detect_all_subnets(self) -> List[str]:
        """Detecta todas las subredes /24 activas en la interfaz"""
        subnets = set()
        subnets.add(self.subnet)
        subnets.add("192.168.1")
        try:
            cmd = ["ip", "route"]
            output = subprocess.check_output(cmd, stderr=subprocess.DEVNULL, timeout=1.0).decode("utf-8")
            for line in output.splitlines():
                match = re.search(r"(\d{1,3}\.\d{1,3}\.\d{1,3})\.\d{1,3}/\d+", line)
                if match:
                    prefix = match.group(1)
                    if not prefix.startswith("127."):
                        subnets.add(prefix)
        except Exception:
            pass
        return list(subnets)

    def _read_arp_table(self) -> Dict[str, str]:
        """Lee la tabla ARP del kernel (/proc/net/arp, ip neigh y arp.exe de Windows Host)"""
        arp_entries: Dict[str, str] = {}
        
        # 1. /proc/net/arp (Linux nativo / WSL2)
        if os.path.exists("/proc/net/arp"):
            try:
                with open("/proc/net/arp", "r") as f:
                    lines = f.readlines()[1:]
                    for line in lines:
                        parts = line.split()
                        if len(parts) >= 4:
                            ip = parts[0]
                            mac = parts[3].lower()
                            if mac != "00:00:00:00:00:00":
                                arp_entries[ip] = mac
            except Exception:
                pass

        # 2. 'ip neigh'
        try:
            cmd = ["ip", "neigh", "show"]
            output = subprocess.check_output(cmd, stderr=subprocess.DEVNULL, timeout=0.8).decode("utf-8")
            for line in output.splitlines():
                parts = line.split()
                if len(parts) >= 5 and "lladdr" in parts:
                    idx = parts.index("lladdr")
                    ip = parts[0]
                    mac = parts[idx + 1].lower()
                    if mac != "00:00:00:00:00:00":
                        arp_entries[ip] = mac
        except Exception:
            pass

        # 3. arp.exe de Windows Host (crucial en entornos WSL2)
        try:
            out = subprocess.check_output(["arp.exe", "-a"], stderr=subprocess.DEVNULL, timeout=0.8).decode("utf-8", "ignore")
            for line in out.splitlines():
                parts = line.split()
                if len(parts) >= 2:
                    ip, mac = parts[0], parts[1]
                    if re.match(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$", ip) and ("-" in mac or ":" in mac):
                        mac_fmt = mac.replace("-", ":").lower()
                        if mac_fmt != "ff:ff:ff:ff:ff:ff" and not ip.startswith("224.") and not ip.startswith("239."):
                            arp_entries[ip] = mac_fmt
        except Exception:
            pass

        return arp_entries

    def _ping_host(self, ip: str) -> Optional[float]:
        """Realiza un ping rápido de 1 paquete y retorna la latencia en ms"""
        if not re.match(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$", ip):
            return None
        try:
            cmd = ["ping", "-c", "1", "-W", "1", ip]
            start = time.time()
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=0.6)
            elapsed = (time.time() - start) * 1000
            if res.returncode == 0:
                output = res.stdout.decode("utf-8", errors="ignore")
                match = re.search(r"time=([0-9.]+)\s*ms", output)
                if match:
                    return float(match.group(1))
                return round(elapsed, 2)
        except Exception:
            pass
        return None

    def _get_hostname(self, ip: str) -> str:
        """Resuelve el nombre de host de forma segura"""
        try:
            host = socket.gethostbyaddr(ip)[0]
            return host
        except Exception:
            return ""

    def _identify_vendor(self, mac: str) -> str:
        """Determina el fabricante probable a partir de la MAC"""
        if not mac or len(mac) < 8:
            return "Desconocido"
        prefix = mac[:8].lower()
        return KNOWN_OUIS.get(prefix, "Genérico / Desconocido")

    def _classify_role(self, ip: str, mac: str, hostname: str, vendor: str) -> Dict[str, Any]:
        """Asigna el rol y estado DDS del dispositivo en el ecosistema ROS 2 / Red"""
        h_lower = hostname.lower()
        domain_label = f"Domain {self.target_domain}"
        
        if ip == self.gateway_ip or ip == "192.168.1.1" or ip.endswith(".1"):
            return {
                "role": "router",
                "label": f"Router Gateway ({ip})",
                "icon": "router",
                "is_dds_active": False,
                "dds_protocol": "Infraestructura Red"
            }
        elif ip == self.local_ip:
            return {
                "role": "host",
                "label": "PC Principal (Host / Control ROS 2)",
                "icon": "desktop",
                "is_dds_active": True,
                "dds_protocol": f"ROS 2 DDS ({domain_label}) & Micro-ROS Agent"
            }
        elif ip == "192.168.1.10" or "kinova" in h_lower or "kortex" in h_lower:
            return {
                "role": "robot",
                "label": "Robot Kinova Gen3 (Brazo 7-DOF)",
                "icon": "robot",
                "is_dds_active": True,
                "dds_protocol": f"ROS 2 DDS ({domain_label})"
            }
        elif "espressif" in vendor.lower() or "esp32" in h_lower or "microros" in h_lower:
            return {
                "role": "esp32",
                "label": f"ESP32 Micro-ROS Node ({ip})",
                "icon": "microchip",
                "is_dds_active": True,
                "dds_protocol": "Micro-ROS Client (UDP 8888)"
            }
        elif "raspberry" in vendor.lower() or "burger" in h_lower or "turtlebot" in h_lower or "pi" in h_lower:
            return {
                "role": "robot",
                "label": f"TurtleBot / Robot Pi ({ip})",
                "icon": "robot",
                "is_dds_active": True,
                "dds_protocol": f"ROS 2 DDS ({domain_label})"
            }
        else:
            return {
                "role": "device",
                "label": f"Dispositivo de Red ({ip})",
                "icon": "wifi",
                "is_dds_active": False,
                "dds_protocol": "Tráfico Estándar (TCP/IP)"
            }

    def scan_network(self, full_sweep: bool = False) -> List[Dict[str, Any]]:
        """
        Escanea la red combinando la tabla ARP del kernel, ARP de Windows Host
        y sondeos ultra-rápidos de hilos paralelos.
        """
        arp_data = self._read_arp_table()
        target_ips = set(arp_data.keys())
        
        # Siempre incluir Gateway, PC local y Robot Kinova
        target_ips.add(self.gateway_ip)
        target_ips.add(self.local_ip)
        target_ips.add("192.168.1.10")

        # IPs comunes de robots y ESP32s
        active_subnets = self._detect_all_subnets()
        for snet in active_subnets:
            for h in [1, 10, 100, 101, 102, 103, 104, 105, 110, 120, 150, 200]:
                target_ips.add(f"{snet}.{h}")

        # Si se solicita barrido masivo, añadir rango 1-254
        if full_sweep:
            for snet in active_subnets:
                for h in range(1, 255):
                    target_ips.add(f"{snet}.{h}")

        devices = []
        
        def probe(ip: str):
            first_octet = int(ip.split(".")[0]) if re.match(r"^\d{1,3}\.", ip) else 0
            if first_octet >= 224 or ip.endswith(".255") or ip == "255.255.255.255":
                return None

            latency = self._ping_host(ip)
            if latency is not None or ip == self.local_ip or ip in arp_data:
                mac = arp_data.get(ip, "Local" if ip == self.local_ip else "--")
                vendor = self._identify_vendor(mac)
                hostname = self._get_hostname(ip)
                role_info = self._classify_role(ip, mac, hostname, vendor)
                
                return {
                    "ip": ip,
                    "mac": mac.upper() if mac != "--" else "N/A",
                    "hostname": hostname or "Desconocido",
                    "vendor": vendor,
                    "role": role_info["role"],
                    "label": role_info["label"],
                    "icon": role_info["icon"],
                    "is_dds_active": role_info["is_dds_active"],
                    "dds_protocol": role_info["dds_protocol"],
                    "latency_ms": latency if latency is not None else (0.1 if ip == self.local_ip else 1.5),
                    "status": "online",
                    "last_seen": time.strftime("%H:%M:%S")
                }
            return None

        # Sondeo masivo paralelo
        with ThreadPoolExecutor(max_workers=80) as executor:
            results = executor.map(probe, list(target_ips))
            for res in results:
                if res:
                    devices.append(res)

        def sort_key(d):
            order = {"router": 0, "host": 1, "robot": 2, "esp32": 3, "device": 4}
            return (order.get(d["role"], 5), socket.inet_aton(d["ip"]) if re.match(r"^\d+\.\d+\.\d+\.\d+$", d["ip"]) else b"")

        devices.sort(key=sort_key)
        self.cached_devices = devices
        self.last_scan_time = time.time()
        return devices


if __name__ == "__main__":

    scanner = DeviceScanner()
    print(f"Gateway: {scanner.gateway_ip}, Local IP: {scanner.local_ip}, Subnet: {scanner.subnet}.0/24")
    print("Escaneando red...")
    devs = scanner.scan_network(full_sweep=True)
    print(f"Encontrados {len(devs)} dispositivos:")
    for d in devs:
        print(f"  [{d['role'].upper()}] {d['ip']:15} | MAC: {d['mac']:17} | Ping: {d['latency_ms']:5.1f}ms | {d['label']}")
