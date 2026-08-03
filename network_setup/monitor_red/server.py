#!/usr/bin/env python3
"""
server.py
Servidor Web local de alto rendimiento y bajo consumo para el
Monitor de Red Híbrido (ROS 2 + Micro-ROS + Router AX12).
Provee API REST, streaming Server-Sent Events (SSE) y sirve la interfaz Web.
"""

import argparse
import json
import os
import socket
import struct
import sys
import threading
import time
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from device_scanner import DeviceScanner
from traffic_sniffer import TrafficSniffer

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")

# Inicialización de servicios
scanner = DeviceScanner()
sniffer = TrafficSniffer(gateway_ip=scanner.gateway_ip)


def test_udp_multicast(group="239.255.0.1", port=7400, timeout_sec=1.5) -> dict:
    """Realiza una prueba rápida de envío y recepción de UDP Multicast"""
    received = False
    test_msg = f"ROS2_PING_{int(time.time()*1000)}".encode("utf-8")
    
    try:
        # Socket receptor
        rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        rx_sock.bind(("", port))
        
        mreq = struct.pack("4sl", socket.inet_aton(group), socket.INADDR_ANY)
        rx_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        rx_sock.settimeout(timeout_sec)

        # Socket emisor
        tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        tx_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        
        start_time = time.time()
        tx_sock.sendto(test_msg, (group, port))

        try:
            data, addr = rx_sock.recvfrom(1024)
            elapsed_ms = (time.time() - start_time) * 1000
            if data == test_msg:
                received = True
                return {
                    "success": True,
                    "message": f"Multicast UDP recibido correctamente desde {addr[0]}",
                    "latency_ms": round(elapsed_ms, 2)
                }
        except socket.timeout:
            pass
        finally:
            rx_sock.close()
            tx_sock.close()
            
    except Exception as e:
        return {"success": False, "message": f"Error en socket multicast: {str(e)}", "latency_ms": 0.0}

    return {
        "success": False,
        "message": "Timeout: No se recibió eco multicast (Revisar AP Isolation en Router o Firewall)",
        "latency_ms": 0.0
    }


class NetworkMonitorHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=STATIC_DIR, **kwargs)

    def _set_json_headers(self, status=HTTPStatus.OK):
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

    def do_OPTIONS(self):
        self.send_response(HTTPStatus.NO_CONTENT)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        # 1. API: Estado general del entorno ROS 2 y Red
        if path == "/api/status":
            self._set_json_headers()
            status_data = {
                "environment": {
                    "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "42 (Default proyecto)"),
                    "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp (Recomendado)"),
                    "ROS_AUTOMATIC_DISCOVERY_RANGE": os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET")
                },
                "network": {
                    "gateway_ip": scanner.gateway_ip,
                    "local_ip": scanner.local_ip,
                    "subnet": f"{scanner.subnet}.0/24"
                },
                "traffic_summary": sniffer.current_metrics
            }
            self.wfile.write(json.dumps(status_data, ensure_ascii=False).encode("utf-8"))
            return

        # 2. API: Lista de dispositivos descubiertos
        elif path == "/api/devices":
            self._set_json_headers()
            # Si aún no hay dispositivos escaneados, ejecutar escaneo inicial
            if not scanner.cached_devices:
                scanner.scan_network(full_sweep=False)
            
            resp = {
                "count": len(scanner.cached_devices),
                "last_scan": time.strftime("%H:%M:%S", time.localtime(scanner.last_scan_time)) if scanner.last_scan_time else "--",
                "devices": scanner.cached_devices
            }
            self.wfile.write(json.dumps(resp, ensure_ascii=False).encode("utf-8"))
            return

        # 3. API: Snapshot de tráfico en tiempo real e historial
        elif path == "/api/traffic":
            self._set_json_headers()
            snapshot = sniffer.get_snapshot()
            self.wfile.write(json.dumps(snapshot, ensure_ascii=False).encode("utf-8"))
            return

        # 4. API: Streaming Server-Sent Events (SSE) en vivo
        elif path == "/api/events":
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            try:
                while True:
                    snap = sniffer.get_snapshot()
                    payload = json.dumps({
                        "traffic": snap["current"],
                        "devices_count": len(scanner.cached_devices),
                        "timestamp": time.strftime("%H:%M:%S")
                    })
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(1.0)
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        # Servir archivos estáticos por defecto
        super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        # 1. API: Forzar re-escaneo completo de la subred
        if path == "/api/scan":
            self._set_json_headers()
            devices = scanner.scan_network(full_sweep=True)
            resp = {
                "status": "success",
                "count": len(devices),
                "devices": devices,
                "timestamp": time.strftime("%H:%M:%S")
            }
            self.wfile.write(json.dumps(resp, ensure_ascii=False).encode("utf-8"))
            return

        # 2. API: Prueba de UDP Multicast
        elif path == "/api/test_multicast":
            self._set_json_headers()
            result = test_udp_multicast()
            self.wfile.write(json.dumps(result, ensure_ascii=False).encode("utf-8"))
            return

        self.send_error(HTTPStatus.NOT_FOUND, "Endpoint no encontrado")

    def log_message(self, format, *args):
        """Silenciar logs continuos de SSE para mantener la terminal limpia"""
        if "/api/events" not in format and "/api/traffic" not in format:
            super().log_message(format, *args)


def run_server(host="127.0.0.1", port=8080):
    # Iniciar escaneo inicial en segundo plano
    threading.Thread(target=lambda: scanner.scan_network(full_sweep=True), daemon=True).start()
    
    # Iniciar recolector de tráfico en segundo plano (cada 1s)
    sniffer.start_background_collector(interval=1.0)

    server = ThreadingHTTPServer((host, port), NetworkMonitorHandler)
    url = f"http://{host}:{port}"
    print(f"\n=======================================================")
    print(f" 🚀 Monitor de Red Híbrido ROS 2 & Router Activo")
    print(f" 🌐 Interfaz Web disponible en: {url}")
    print(f" 📡 Gateway detectado: {scanner.gateway_ip} | IP Local: {scanner.local_ip}")
    print(f"=======================================================\n")
    print("Presiona Ctrl+C para detener el servidor.\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nDeteniendo monitor de red...")
    finally:
        sniffer.stop_background_collector()
        server.server_close()
        print("Servidor detenido correctamente.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Monitor de Red Híbrido para ROS 2 y Router")
    parser.add_argument("--host", default="127.0.0.1", help="Dirección IP de escucha (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8080, help="Puerto HTTP (default: 8080)")
    args = parser.parse_args()

    run_server(host=args.host, port=args.port)
