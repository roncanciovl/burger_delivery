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
        self.last_scan_time = 0.0
        self.cached_devices: List[Dict[str, Any]] = []

    def _get_default_gateway(self) -> str:
        """Obtiene la IP del Gateway predeterminado (Router)"""
        try:
            cmd = ["ip", "route"]
            output = subprocess.check_output(cmd, stderr=subprocess.DEVNULL, timeout=2).decode("utf-8")
            for line in output.splitlines():
                if line.startswith("default"):
                    parts = line.split()
                    if len(parts) >= 3 and parts[1] == "via":
                        return parts[2]
        except Exception:
            pass
        return "192.168.1.1"

    def _get_local_ip(self) -> str:
        """Obtiene la IP local de la interfaz principal"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # No envía paquete real, solo conecta a nivel de socket
            s.connect((self.gateway_ip if self.gateway_ip else "8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"

    def _detect_subnet(self) -> str:
        """Calcula el prefijo de subred clase C (ej. 192.168.1)"""
        ip_parts = self.local_ip.split(".")
        if len(ip_parts) == 4 and ip_parts[0] != "127":
            return f"{ip_parts[0]}.{ip_parts[1]}.{ip_parts[2]}"
        return "192.168.1"

    def _read_arp_table(self) -> Dict[str, str]:
        """Lee la tabla ARP del kernel (/proc/net/arp o ip neigh)"""
        arp_entries: Dict[str, str] = {}
        
        # 1. Intento por /proc/net/arp (Linux nativo / WSL2)
        if os.path.exists("/proc/net/arp"):
            try:
                with open("/proc/net/arp", "r") as f:
                    lines = f.readlines()[1:]  # Omitir encabezado
                    for line in lines:
                        parts = line.split()
                        if len(parts) >= 4:
                            ip = parts[0]
                            mac = parts[3].lower()
                            if mac != "00:00:00:00:00:00":
                                arp_entries[ip] = mac
            except Exception:
                pass

        # 2. Intento complementario con 'ip neigh'
        try:
            cmd = ["ip", "neigh", "show"]
            output = subprocess.check_output(cmd, stderr=subprocess.DEVNULL, timeout=2).decode("utf-8")
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

        return arp_entries

    def _ping_host(self, ip: str) -> Optional[float]:
        """Realiza un ping rápido de 1 paquete y retorna la latencia en ms"""
        # Validación estricta de formato IPv4 para seguridad
        if not re.match(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$", ip):
            return None
        try:
            cmd = ["ping", "-c", "1", "-W", "1", ip]
            start = time.time()
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=1.5)
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

    def _classify_role(self, ip: str, mac: str, hostname: str, vendor: str) -> Dict[str, str]:
        """Asigna el rol del dispositivo en el ecosistema ROS 2 / Red"""
        h_lower = hostname.lower()
        
        if ip == self.gateway_ip:
            return {
                "role": "router",
                "label": "Router WiFi (TP-Link AX12 / Gateway)",
                "icon": "router"
            }
        elif ip == self.local_ip:
            return {
                "role": "host",
                "label": "PC Principal (Host / Control ROS 2)",
                "icon": "desktop"
            }
        elif "espressif" in vendor.lower() or "esp32" in h_lower or "microros" in h_lower:
            return {
                "role": "esp32",
                "label": "ESP32 (Micro-ROS Node)",
                "icon": "microchip"
            }
        elif "raspberry" in vendor.lower() or "burger" in h_lower or "turtlebot" in h_lower or "pi" in h_lower:
            return {
                "role": "robot",
                "label": "TurtleBot Burger (Robot Pi)",
                "icon": "robot"
            }
        else:
            return {
                "role": "device",
                "label": "Dispositivo de Red",
                "icon": "wifi"
            }

    def scan_network(self, full_sweep: bool = False) -> List[Dict[str, Any]]:
        """
        Escanea la red combinando la tabla ARP del kernel y opcionalmente
        un barrido rápido de las IPs más comunes (1 a 30, 100 a 115, 200 a 210).
        """
        arp_data = self._read_arp_table()
        target_ips = set(arp_data.keys())
        
        # Siempre incluir Gateway y PC local
        target_ips.add(self.gateway_ip)
        target_ips.add(self.local_ip)

        # Si se solicita barrido rápido, probar las IPs comunes donde se colocan robots y ESP32s
        if full_sweep or len(target_ips) <= 2:
            base = self.subnet
            # Rango común de DHCP y reservas estáticas de laboratorio
            common_hosts = [1, 100, 101, 102, 103, 104, 105, 106, 110, 120, 150, 200]
            for h in common_hosts:
                target_ips.add(f"{base}.{h}")

        devices = []
        
        def probe(ip: str):
            latency = self._ping_host(ip)
            if latency is not None or ip == self.local_ip:
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
                    "latency_ms": latency if latency is not None else 0.1,
                    "status": "online",
                    "last_seen": time.strftime("%H:%M:%S")
                }
            return None

        # Ejecutar sondeos de ping en paralelo para responder en < 1 segundo
        with ThreadPoolExecutor(max_workers=20) as executor:
            results = executor.map(probe, list(target_ips))
            for res in results:
                if res:
                    devices.append(res)

        # Ordenar: Router primero, luego PC Host, luego Robots/ESP32, luego otros
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
