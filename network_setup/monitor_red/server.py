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
import subprocess
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from device_scanner import DeviceScanner
from traffic_sniffer import TrafficSniffer

STATIC_DIR = os.path.join(CURRENT_DIR, "static")


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


from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

class NetworkMonitorHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def _send_bytes(self, data: bytes, content_type: str, status=HTTPStatus.OK):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(data)

    def _set_json_headers(self, data_dict: dict, status=HTTPStatus.OK):
        payload = json.dumps(data_dict, ensure_ascii=False).encode("utf-8")
        self._send_bytes(payload, "application/json; charset=utf-8", status)

    def do_HEAD(self):
        self.do_GET()

    def do_OPTIONS(self):
        self.send_response(HTTPStatus.NO_CONTENT)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS, HEAD")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Connection", "close")
        self.end_headers()


    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        # 1. Archivos estáticos principales
        if path in ("/", "/index.html"):
            index_path = os.path.join(STATIC_DIR, "index.html")
            try:
                with open(index_path, "rb") as f:
                    self._send_bytes(f.read(), "text/html; charset=utf-8")
            except Exception as e:
                self._send_bytes(f"Error cargando index.html: {e}".encode("utf-8"), "text/plain", HTTPStatus.INTERNAL_SERVER_ERROR)
            return

        elif path == "/app.css":
            css_path = os.path.join(STATIC_DIR, "app.css")
            try:
                with open(css_path, "rb") as f:
                    self._send_bytes(f.read(), "text/css; charset=utf-8")
            except Exception as e:
                self._send_bytes(b"", "text/css; charset=utf-8", HTTPStatus.NOT_FOUND)
            return

        elif path == "/app.js":
            js_path = os.path.join(STATIC_DIR, "app.js")
            try:
                with open(js_path, "rb") as f:
                    self._send_bytes(f.read(), "application/javascript; charset=utf-8")
            except Exception as e:
                self._send_bytes(b"", "application/javascript; charset=utf-8", HTTPStatus.NOT_FOUND)
            return

        # 2. API: Estado general del entorno ROS 2 y Red
        elif path == "/api/status":
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
            self._set_json_headers(status_data)
            return

        # 3. API: Lista de dispositivos descubiertos
        elif path == "/api/devices":
            resp = {
                "count": len(scanner.cached_devices),
                "last_scan": time.strftime("%H:%M:%S", time.localtime(scanner.last_scan_time)) if scanner.last_scan_time else "--",
                "devices": scanner.cached_devices
            }
            self._set_json_headers(resp)
            return

        # 4. API: Snapshot de tráfico en tiempo real e historial
        elif path == "/api/traffic":
            snapshot = sniffer.get_snapshot()
            self._set_json_headers(snapshot)
            return

        # 5. API: Estado del Benchmark de Telemetría
        elif path == "/api/benchmark/status":
            b_status = sniffer.get_benchmark_status()
            self._set_json_headers(b_status)
            return

        # 6. API: Descarga del último CSV de Benchmark
        elif path == "/api/benchmark/download":
            csv_content = sniffer.get_latest_benchmark_csv_content()
            if csv_content:
                csv_bytes = csv_content.encode("utf-8")
                filename = os.path.basename(sniffer.last_saved_benchmark_file) if sniffer.last_saved_benchmark_file else "benchmark.csv"
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/csv; charset=utf-8")
                self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
                self.send_header("Content-Length", str(len(csv_bytes)))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(csv_bytes)
                return
            else:
                self._set_json_headers({"status": "error", "message": "No hay archivo CSV disponible para descargar"}, status=HTTPStatus.NOT_FOUND)
                return

        self._send_bytes(b"Recurso no encontrado", "text/plain", HTTPStatus.NOT_FOUND)

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        # 1. API: Forzar re-escaneo completo de la subred
        if path == "/api/scan":
            devices = scanner.scan_network(full_sweep=True)
            resp = {
                "status": "success",
                "count": len(devices),
                "devices": devices,
                "timestamp": time.strftime("%H:%M:%S")
            }
            self._set_json_headers(resp)
            return

        # 2. API: Prueba de UDP Multicast
        elif path == "/api/test_multicast":
            result = test_udp_multicast()
            self._set_json_headers(result)
            return

        # 3. API: Actualización de Configuración (ROS_DOMAIN_ID)
        elif path == "/api/config":
            try:
                length = int(self.headers.get("Content-Length", 0))
                raw_body = self.rfile.read(length).decode("utf-8")
                data = json.loads(raw_body)
                if "ros_domain_id" in data:
                    domain_id = int(data["ros_domain_id"])
                    scanner.set_target_domain(domain_id)
                    threading.Thread(target=lambda: scanner.scan_network(full_sweep=False), daemon=True).start()
                    self._set_json_headers({"status": "success", "ROS_DOMAIN_ID": domain_id})
                    return
            except Exception as e:
                self._set_json_headers({"status": "error", "message": str(e)}, status=HTTPStatus.BAD_REQUEST)
                return

        # 4. API: Iniciar Benchmark de Telemetría
        elif path == "/api/benchmark/start":
            try:
                length = int(self.headers.get("Content-Length", 0))
                raw_body = self.rfile.read(length).decode("utf-8") if length > 0 else "{}"
                data = json.loads(raw_body) if raw_body else {}
                
                session_name = data.get("session_name", "ensayo_qos")
                scenario = data.get("scenario", "Linea_Base_WiFi6")
                
                res = sniffer.start_benchmark(session_name=session_name, scenario=scenario)
                self._set_json_headers(res)
                return
            except Exception as e:
                self._set_json_headers({"status": "error", "message": str(e)}, status=HTTPStatus.INTERNAL_SERVER_ERROR)
                return

        # 5. API: Detener Benchmark y Guardar CSV
        elif path == "/api/benchmark/stop":
            try:
                res = sniffer.stop_benchmark()
                self._set_json_headers(res)
                return
            except Exception as e:
                self._set_json_headers({"status": "error", "message": str(e)}, status=HTTPStatus.INTERNAL_SERVER_ERROR)
                return


        self._send_bytes(b"Endpoint no encontrado", "text/plain", HTTPStatus.NOT_FOUND)

    def log_message(self, format, *args):
        """Silenciar logs continuos de telemetría para no saturar consola"""
        pass



def open_browser(url: str):
    """Abre el navegador en el puerto activo según el entorno (WSL2 o Linux nativo)"""
    def _open():
        time.sleep(0.4)
        is_wsl = False
        if os.path.exists("/proc/version"):
            try:
                with open("/proc/version", "r") as f:
                    if "microsoft" in f.read().lower():
                        is_wsl = True
            except Exception:
                pass
        
        if is_wsl:
            for explorer_path in ["explorer.exe", "/mnt/c/WINDOWS/explorer.exe", "/mnt/c/Windows/explorer.exe"]:
                try:
                    subprocess.Popen([explorer_path, url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    return
                except Exception:
                    continue
        
        try:
            subprocess.Popen(["xdg-open", url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

    threading.Thread(target=_open, daemon=True).start()


def run_server(host="0.0.0.0", port=8080):
    ThreadingHTTPServer.allow_reuse_address = True

    # Intentar enlazar en el puerto solicitado o buscar el siguiente disponible
    server = None
    actual_port = port
    for attempt in range(10):
        try:
            server = ThreadingHTTPServer((host, actual_port), NetworkMonitorHandler)
            break
        except OSError as e:
            if e.errno == 98:  # Address already in use
                actual_port = port + attempt + 1
            else:
                raise e

    if server is None:
        print(f"Error: No se pudo enlazar el servidor en el rango de puertos {port}-{port+10}")
        sys.exit(1)

    # Iniciar escaneo inicial en segundo plano
    threading.Thread(target=lambda: scanner.scan_network(full_sweep=True), daemon=True).start()
    
    # Iniciar recolector de tráfico en segundo plano (cada 1s)
    sniffer.start_background_collector(interval=1.0)

    display_host = "localhost" if host in ("0.0.0.0", "127.0.0.1") else host
    url = f"http://{display_host}:{actual_port}"
    print(f"\n=======================================================")
    print(f" 🚀 Monitor de Red Híbrido ROS 2 & Router Activo")
    print(f" 🌐 Interfaz Web disponible en: {url}")
    print(f" 📡 Gateway detectado: {scanner.gateway_ip} | IP Local: {scanner.local_ip}")
    print(f"=======================================================\n")
    print("Presiona Ctrl+C para detener el servidor.\n")

    # Abrir navegador automáticamente con la URL y puerto exacto
    open_browser(url)


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
    parser.add_argument("--host", default="0.0.0.0", help="Dirección IP de escucha (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8080, help="Puerto HTTP (default: 8080)")
    args = parser.parse_args()

    run_server(host=args.host, port=args.port)


