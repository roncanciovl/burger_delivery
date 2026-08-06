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
    # TP-Link (Routers & Adaptadores)
    "50:d4:f7": "TP-Link", "98:25:4a": "TP-Link", "ec:17:2f": "TP-Link",
    "14:eb:b6": "TP-Link", "30:de:4b": "TP-Link", "54:af:97": "TP-Link",
    "70:4f:57": "TP-Link", "84:16:f9": "TP-Link", "b0:4e:26": "TP-Link",
    "c0:06:c3": "TP-Link", "c0:25:e9": "TP-Link", "c4:71:54": "TP-Link",
    "d8:07:b6": "TP-Link", "e8:48:b8": "TP-Link", "f4:f2:6d": "TP-Link",
    # Kinova Robotics
    "00:11:22": "Kinova Robotics", "00:23:7d": "Kinova Robotics", "00:50:c2": "Kinova Robotics", "70:b3:d5": "Kinova Robotics",
    # Intel (PC / NUC / Workstations)
    "00:15:00": "Intel PC", "00:1b:21": "Intel PC", "00:21:6a": "Intel PC", "00:1e:67": "Intel PC",
    "68:05:ca": "Intel PC", "80:86:f2": "Intel PC", "a0:36:9f": "Intel PC", "e4:a8:df": "Intel PC",
    "ec:0d:9a": "Intel PC", "f8:e4:3b": "Intel PC", "7c:57:58": "Intel PC", "08:8e:90": "Intel PC",
    "90:65:84": "Intel PC / Estación", "a0:b3:39": "Intel PC / Estación", "f4:7b:09": "Intel PC / Estación",
    "9c:67:d6": "Intel PC / Estación", "d8:f3:bc": "Intel PC / Estación", "00:45:e2": "Intel / Realtek PC",
    # Dell
    "00:14:22": "Dell Workstation", "14:fe:b5": "Dell PC", "18:66:da": "Dell PC", "24:b6:fd": "Dell PC",
    "34:17:eb": "Dell PC", "74:86:7a": "Dell PC", "b8:2a:72": "Dell PC", "d4:be:d9": "Dell PC",
    # Lenovo / ThinkPad
    "00:59:07": "Lenovo ThinkPad", "40:8d:5c": "Lenovo PC", "54:ee:75": "Lenovo PC", "60:02:92": "Lenovo PC",
    "70:72:cf": "Lenovo PC", "88:70:8c": "Lenovo PC", "a4:bb:6d": "Lenovo PC", "cc:52:af": "Lenovo PC",
    # HP (Hewlett-Packard)
    "00:08:02": "HP Workstation", "00:0b:cd": "HP PC", "00:11:0a": "HP PC", "00:17:a4": "HP PC",
    "00:21:5a": "HP PC", "28:92:4a": "HP PC", "3c:d9:2b": "HP PC", "9c:b6:54": "HP PC",
    # Apple (MacBook / Mac Mini / Control Station)
    "90:e8:68": "Apple Mac / Control Station", "a4:83:e7": "Apple Mac", "ac:bc:32": "Apple Mac",
    "b0:34:95": "Apple Mac", "bc:52:b7": "Apple Mac", "c8:69:cd": "Apple Mac", "cc:08:8d": "Apple Mac",
    "d0:81:7a": "Apple Mac", "dc:a9:04": "Apple Mac", "e0:ac:cb": "Apple Mac", "f0:18:98": "Apple Mac",
    # Microsoft Surface / Host
    "00:15:5d": "Microsoft Host / WSL", "28:18:78": "Microsoft Surface", "60:45:bd": "Microsoft PC",
    # Realtek / ASUS / Gigabyte (Placas Base PC)
    "00:e0:4c": "Realtek PC NIC", "52:54:00": "QEMU/KVM Virtual PC", "00:1e:8c": "ASUS Workstation",
    "08:60:6e": "ASUS PC", "10:7b:44": "ASUS PC", "ac:22:0b": "ASUS PC", "e4:aa:ea": "PC Host / NIC",
    "cc:47:40": "PC / NIC Host", "d2:dc:01": "Dispositivo Red", "3e:98:92": "Dispositivo Red"
}

class DeviceScanner:
    def __init__(self, subnet: Optional[str] = None):
        self.gateway_ip = self._get_default_gateway()
        self.local_ip = self._get_local_ip()
        self.subnet = subnet or self._detect_subnet()
        self.target_domain = int(os.environ.get("ROS_DOMAIN_ID", 42))
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
                "dds_protocol": f"ROS 2 DDS (Domain {self.target_domain}) & Micro-ROS Agent",
                "latency_ms": 0.1,
                "status": "online",
                "last_seen": time.strftime("%H:%M:%S")
            }
        ]

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

    def _get_local_domains(self) -> List[int]:
        """Detecta dominios ROS 2 con sockets UDP abiertos en este host"""
        domains = set()
        env_d = os.environ.get("ROS_DOMAIN_ID")
        if env_d is not None and env_d.isdigit():
            domains.add(int(env_d))
        
        for proc_file in ["/proc/net/udp", "/proc/net/udp6"]:
            if os.path.exists(proc_file):
                try:
                    with open(proc_file, "r") as f:
                        for line in f.readlines()[1:]:
                            parts = line.split()
                            if len(parts) >= 2 and ":" in parts[1]:
                                port = int(parts[1].split(":")[-1], 16)
                                if 7400 <= port <= 32000:
                                    d = (port - 7400) // 250
                                    if 0 <= d <= 230:
                                        domains.add(d)
                except Exception:
                    pass
        if not domains:
            domains.add(self.target_domain)
        return sorted(list(domains))

    def _classify_role(self, ip: str, mac: str, hostname: str, vendor: str, detected_domains: Optional[List[int]] = None) -> Dict[str, Any]:
        """Asigna el rol y estado DDS del dispositivo en el ecosistema ROS 2 / Red con discriminación de Domain ID"""
        h_lower = hostname.lower()
        v_lower = vendor.lower()
        local_domains = self._get_local_domains()
        
        # Determinar lista de dominios para este host específico
        host_domains: List[int] = []
        if detected_domains:
            host_domains = list(detected_domains)
        elif ip == self.local_ip:
            host_domains = local_domains

        if ip == self.gateway_ip or ip == "192.168.1.1" or ip.endswith(".1"):
            return {
                "role": "router",
                "label": f"Router Gateway ({ip})",
                "icon": "router",
                "is_dds_active": False,
                "domain_id": None,
                "dds_domains": [],
                "dds_protocol": "Infraestructura Red"
            }
        elif ip == self.local_ip:
            dom_text = ", ".join(str(d) for d in local_domains) if local_domains else str(self.target_domain)
            return {
                "role": "host",
                "label": "PC Principal (Host / Control ROS 2)",
                "icon": "desktop",
                "is_dds_active": True,
                "domain_id": local_domains[0] if local_domains else self.target_domain,
                "dds_domains": local_domains,
                "dds_protocol": f"ROS 2 DDS (Domain {dom_text}) & Micro-ROS Agent"
            }
        elif ip == "192.168.1.10" or "kinova" in h_lower or "kortex" in h_lower:
            dom_val = host_domains[0] if host_domains else self.target_domain
            return {
                "role": "robot",
                "label": "Robot Kinova Gen3 (Brazo 7-DOF)",
                "icon": "robot",
                "is_dds_active": True,
                "domain_id": dom_val,
                "dds_domains": host_domains or [dom_val],
                "dds_protocol": f"ROS 2 DDS (Domain {dom_val})"
            }
        elif "espressif" in v_lower or "esp32" in h_lower or "microros" in h_lower:
            return {
                "role": "esp32",
                "label": f"ESP32 Micro-ROS Node ({ip})",
                "icon": "microchip",
                "is_dds_active": True,
                "domain_id": self.target_domain,
                "dds_domains": [self.target_domain],
                "dds_protocol": f"Micro-ROS Client (UDP 8888 -> Dom {self.target_domain})"
            }
        elif "raspberry" in v_lower or "burger" in h_lower or "turtlebot" in h_lower or "pi" in h_lower:
            dom_val = host_domains[0] if host_domains else self.target_domain
            return {
                "role": "robot",
                "label": f"TurtleBot / Robot Pi ({ip})",
                "icon": "robot",
                "is_dds_active": True,
                "domain_id": dom_val,
                "dds_domains": host_domains or [dom_val],
                "dds_protocol": f"ROS 2 DDS (Domain {dom_val})"
            }
        elif self._is_computer_node(hostname, vendor):
            # Computadores / Laptops / Workstations en la red ROS 2 DDS
            host_title = hostname if hostname and hostname != "Desconocido" else f"Estación / PC ({ip})"
            
            if host_domains:
                dom_str = ", ".join(str(d) for d in host_domains)
                dom_label = f"Domain {dom_str}" if len(host_domains) == 1 else f"Multidominio: {dom_str}"
                primary_dom = host_domains[0]
            else:
                primary_dom = self.target_domain
                host_domains = [self.target_domain]
                dom_label = f"Domain {self.target_domain} (Proyecto Burger-Cell)"

            return {
                "role": "host",
                "label": f"PC / Nodo ROS 2 ({host_title})",
                "icon": "desktop",
                "is_dds_active": True,
                "domain_id": primary_dom,
                "dds_domains": host_domains,
                "dds_protocol": f"ROS 2 DDS ({dom_label})"
            }
        else:
            if host_domains:
                dom_str = ", ".join(str(d) for d in host_domains)
                return {
                    "role": "host",
                    "label": f"Nodo ROS 2 ({ip})",
                    "icon": "desktop",
                    "is_dds_active": True,
                    "domain_id": host_domains[0],
                    "dds_domains": host_domains,
                    "dds_protocol": f"ROS 2 DDS (Domain {dom_str})"
                }
            return {
                "role": "device",
                "label": f"Dispositivo de Red ({ip})",
                "icon": "wifi",
                "is_dds_active": False,
                "domain_id": None,
                "dds_domains": [],
                "dds_protocol": "Tráfico Estándar (TCP/IP)"
            }

    def _is_computer_node(self, hostname: str, vendor: str) -> bool:
        """Determina si un host es una estación o computador en la subred ROS 2"""
        h = (hostname or "").lower()
        v = (vendor or "").lower()
        pc_keys = ["pc", "desktop", "laptop", "workstation", "ubuntu", "linux", "station", "node", "ros", "nuc", "jetson", "orin", "xavier", "thinkpad", "surface", "macbook", "host", "roncanciovl", "burger"]
        vendor_keys = ["intel", "dell", "lenovo", "hp", "hewlett", "apple", "asus", "asustek", "microsoft", "realtek", "qemu", "nic"]
        return any(k in h for k in pc_keys) or any(vk in v for vk in vendor_keys)

    def scan_network(self, full_sweep: bool = False, ip_domains_map: Optional[Dict[str, List[int]]] = None) -> List[Dict[str, Any]]:
        """
        Escanea la red combinando la tabla ARP del kernel, ARP de Windows Host
        y sondeos ultra-rápidos de hilos paralelos con discriminación de Domain ID.
        """
        domains_map = ip_domains_map or {}
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
            if latency is not None or ip == self.local_ip or ip in arp_data or ip in domains_map:
                mac = arp_data.get(ip, "Local" if ip == self.local_ip else "--")
                vendor = self._identify_vendor(mac)
                hostname = self._get_hostname(ip)
                detected_doms = domains_map.get(ip)
                role_info = self._classify_role(ip, mac, hostname, vendor, detected_doms)
                
                return {
                    "ip": ip,
                    "mac": mac.upper() if mac != "--" else "N/A",
                    "hostname": hostname or "Desconocido",
                    "vendor": vendor,
                    "role": role_info["role"],
                    "label": role_info["label"],
                    "icon": role_info["icon"],
                    "is_dds_active": role_info["is_dds_active"],
                    "domain_id": role_info.get("domain_id"),
                    "dds_domains": role_info.get("dds_domains", []),
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
