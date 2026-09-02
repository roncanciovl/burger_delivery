#!/usr/bin/env python3
"""
Kinova Gen3 - Visor de cámara en tiempo real
Conexión directa por RTSP sin pasar por ROS 2.

Uso:  python3 test_kinova_camera.py [--ip 192.168.1.10] [--stream color|depth]

------------------------------------------------------------------------------
POR QUÉ 'color' Y 'depth' NO SE ABREN IGUAL
------------------------------------------------------------------------------
El módulo de visión del Kinova Gen3 expone DOS servicios RTSP distintos:

  * rtsp://<ip>/color -> RTP + H.264 estándar.
    Lo entienden tanto FFMPEG como GStreamer.

  * rtsp://<ip>/depth -> RTP con payload propio de GStreamer
    (application/x-rtp, encoding-name=X-GST). Al depayloadear con
    'rtpgstdepay' entrega video/x-raw en formato GRAY16_LE: 16 bits por
    píxel, 480x270, con la DISTANCIA EN MILÍMETROS (no es una imagen RGB).

FFmpeg no implementa el depayloader X-GST, por lo que los intentos
"FFMPEG TCP" y "FFMPEG UDP" SIEMPRE fallan para depth: eso no es un error de
red ni del robot. El stream de profundidad solo se puede abrir con el backend
GStreamer y requiere los plugins 'good' instalados en el sistema.
"""
import cv2
import numpy as np
import argparse
import glob
import socket
import sys
import time
import os


RTSP_PORT = 554

APT_HINT = (
    "sudo apt update && sudo apt install -y \\\n"
    "      gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \\\n"
    "      gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav"
)

# Plugin (archivo .so) -> elemento del pipeline que aporta
GST_REQUIRED_COMMON = {
    'libgstrtsp.so': 'rtspsrc',
    'libgstrtpmanager.so': 'rtpjitterbuffer',
    'libgstrtp.so': 'rtph264depay / rtpgstdepay',
    'libgstapp.so': 'appsink',
    'libgstvideoconvertscale.so': 'videoconvert / videoscale',
}
GST_REQUIRED_COLOR = {
    'libgstlibav.so': 'avdec_h264',
    'libgstvideoparsersbad.so': 'h264parse',
}


# ---------------------------------------------------------------------------
# Diagnóstico por capas (previo a abrir el stream)
# ---------------------------------------------------------------------------
def check_rtsp_port(ip, timeout=3.0):
    """Capa 1-2: ¿responde el servidor RTSP del Kinova en el puerto 554?"""
    host = ip.split('@')[-1]  # admite formato usuario:clave@ip
    try:
        with socket.create_connection((host, RTSP_PORT), timeout=timeout):
            print(f"[INFO] ✓ Puerto RTSP {host}:{RTSP_PORT} accesible")
            return True
    except OSError as exc:
        print(f"[ERROR] ✗ No hay respuesta en {host}:{RTSP_PORT} ({exc})")
        print("[ERROR]   Revise la capa de red antes de seguir:")
        print(f"[ERROR]     ping {host}")
        print("[ERROR]     ip addr show   # la NIC cableada debe estar en 192.168.1.0/24")
        return False


def _gst_plugin_dirs():
    dirs = []
    env_path = os.environ.get('GST_PLUGIN_PATH', '')
    dirs += [d for d in env_path.split(os.pathsep) if d]
    dirs += glob.glob('/usr/lib/*/gstreamer-1.0')
    dirs += ['/usr/lib/gstreamer-1.0']
    return [d for d in dirs if os.path.isdir(d)]


def missing_gst_plugins(stream_type):
    """Devuelve la lista de elementos GStreamer que faltan en el sistema."""
    plugin_dirs = _gst_plugin_dirs()
    if not plugin_dirs:
        return []  # instalación no estándar: dejamos que el pipeline lo diga

    present = set()
    for d in plugin_dirs:
        present.update(os.listdir(d))

    required = dict(GST_REQUIRED_COMMON)
    if stream_type == 'color':
        required.update(GST_REQUIRED_COLOR)

    return [element for so, element in required.items() if so not in present]


def report_missing_plugins(faltantes, stream_type):
    print("[ERROR] ✗ Faltan plugins de GStreamer en este equipo.")
    print(f"[ERROR]   Elementos no disponibles: {', '.join(faltantes)}")
    print("[ERROR]   OpenCV se compiló CON soporte GStreamer, pero solo está")
    print("[ERROR]   instalado 'gstreamer1.0-plugins-base'. Por eso aparece:")
    print('[ERROR]     No URI handler implemented for "rtsp"')
    print("[ERROR]   Instale los plugins y vuelva a ejecutar:\n")
    print(f"      {APT_HINT}\n")
    if stream_type == 'depth':
        print("[ERROR]   El stream 'depth' NO tiene alternativa por FFMPEG:")
        print("[ERROR]   depende de rtpgstdepay, que solo existe en GStreamer.")


# ---------------------------------------------------------------------------
# Backends de captura
# ---------------------------------------------------------------------------
def build_pipeline(rtsp_url, stream_type, latency_ms):
    """Pipeline GStreamer con timeouts explícitos (evita cuelgues sin salida)."""
    src = (
        f"rtspsrc location={rtsp_url} protocols=tcp latency={latency_ms} "
        f"buffer-mode=auto timeout=5000000 tcp-timeout=5000000 drop-on-latency=true"
    )
    if stream_type == 'color':
        return (
            f"{src} "
            f"! rtph264depay ! h264parse ! avdec_h264 "
            f"! videoconvert "
            f"! video/x-raw,format=BGR "
            f"! appsink drop=true max-buffers=1 sync=false"
        )
    # depth: GRAY16_LE nativo (milímetros). NO se convierte a BGR aquí:
    # videoconvert descartaría el byte alto y la imagen saldría casi negra.
    return (
        f"{src} "
        f"! rtpgstdepay "
        f"! videoconvert "
        f"! video/x-raw,format=GRAY16_LE "
        f"! appsink drop=true max-buffers=1 sync=false"
    )


def try_gstreamer(rtsp_url, stream_type, latency_ms):
    """Intentar conexión con GStreamer pipeline."""
    pipeline = build_pipeline(rtsp_url, stream_type, latency_ms)
    print("[INFO] Intentando GStreamer pipeline...")
    print(f"[DEBUG] Pipeline: {pipeline}")

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        ret, _ = cap.read()
        if ret:
            print("[INFO] ✓ Conectado con GStreamer")
            return cap
        cap.release()
    print("[WARN] GStreamer pipeline no funcionó")
    return None


def _try_ffmpeg(rtsp_url, transport):
    print(f"[INFO] Intentando FFMPEG con {transport.upper()}...")
    os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = (
        f'rtsp_transport;{transport}|'
        'fflags;nobuffer|'
        'flags;low_delay|'
        'max_delay;0|'
        'stimeout;5000000|'
        'analyzeduration;100000|'
        'probesize;100000'
    )
    cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ret, _ = cap.read()
        if ret:
            print(f"[INFO] ✓ Conectado con FFMPEG {transport.upper()}")
            return cap
        cap.release()
    print(f"[WARN] FFMPEG {transport.upper()} no funcionó")
    return None


def try_ffmpeg_tcp(rtsp_url):
    return _try_ffmpeg(rtsp_url, 'tcp')


def try_ffmpeg_udp(rtsp_url):
    return _try_ffmpeg(rtsp_url, 'udp')


# ---------------------------------------------------------------------------
# Visualización
# ---------------------------------------------------------------------------
def colorize_depth(frame_mm, max_range_mm):
    """GRAY16_LE (mm) -> mapa de color JET. Los píxeles sin dato quedan negros."""
    valid = frame_mm > 0
    clipped = np.clip(frame_mm, 0, max_range_mm).astype(np.float32)
    scaled = (clipped * (255.0 / max_range_mm)).astype(np.uint8)
    vis = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
    vis[~valid] = (0, 0, 0)
    return vis


def main():
    parser = argparse.ArgumentParser(description='Kinova Camera - Visor en tiempo real')
    parser.add_argument('--ip', type=str, default='192.168.1.10',
                        help='IP del robot (default: 192.168.1.10)')
    parser.add_argument('--stream', type=str, default='color',
                        choices=['color', 'depth'],
                        help='Tipo de stream (default: color)')
    parser.add_argument('--width', type=int, default=640,
                        help='Ancho de la ventana (default: 640)')
    parser.add_argument('--height', type=int, default=480,
                        help='Alto de la ventana (default: 480)')
    parser.add_argument('--latency', type=int, default=30,
                        help='Latencia del jitterbuffer GStreamer en ms (default: 30)')
    parser.add_argument('--max-depth', type=int, default=4000,
                        help='Rango máximo en mm para colorear depth (default: 4000)')
    parser.add_argument('--skip-checks', action='store_true',
                        help='Omitir el diagnóstico previo de red y plugins')
    args = parser.parse_args()

    is_depth = args.stream == 'depth'
    rtsp_url = f"rtsp://{args.ip}/{args.stream}"
    print(f"[INFO] Conectando a: {rtsp_url}")
    print(f"[INFO] Resolución de visualización: {args.width}x{args.height}")
    print("[INFO] Presiona 'q' para salir, 's' para guardar captura")
    print()

    if not args.skip_checks:
        if not check_rtsp_port(args.ip):
            sys.exit(2)
        faltantes = missing_gst_plugins(args.stream)
        if faltantes:
            report_missing_plugins(faltantes, args.stream)
            if is_depth:
                sys.exit(3)
            print("[WARN] Se continuará solo con FFMPEG (válido para color).\n")
        print()

    # 'depth' no pasa por FFMPEG: su payload X-GST no está implementado ahí.
    if is_depth:
        print("[INFO] Stream de profundidad: se usa GStreamer directamente")
        print("[INFO] (FFMPEG no implementa el depayloader X-GST del Kinova)\n")
        attempts = [lambda: try_gstreamer(rtsp_url, 'depth', args.latency)]
    else:
        attempts = [
            lambda: try_ffmpeg_tcp(rtsp_url),
            lambda: try_ffmpeg_udp(rtsp_url),
            lambda: try_gstreamer(rtsp_url, 'color', args.latency),
        ]

    cap = None
    for attempt_fn in attempts:
        cap = attempt_fn()
        if cap is not None:
            break

    if cap is None:
        print(f"\n[ERROR] No se pudo conectar a {rtsp_url}")
        print("[ERROR] Verifica que:")
        print("  1. El robot esté encendido")
        print(f"  2. La IP {args.ip} sea correcta")
        print("  3. La cámara esté habilitada en la Web App http://" + args.ip.split('@')[-1])
        if is_depth:
            print("  4. Los plugins GStreamer estén instalados:\n")
            print(f"      {APT_HINT}")
        sys.exit(1)

    print("\n[INFO] ¡Mostrando imagen! (Presiona 'q' para salir)")

    frame_count = 0
    fps_start = time.time()
    fps_display = 0.0
    capture_count = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Frame perdido, reintentando...")
                time.sleep(0.05)
                continue

            frame_count += 1
            elapsed = time.time() - fps_start
            if elapsed >= 1.0:
                fps_display = frame_count / elapsed
                frame_count = 0
                fps_start = time.time()

            raw = frame
            if is_depth:
                depth_mm = raw if raw.ndim == 2 else raw[:, :, 0]
                h, w = depth_mm.shape[:2]
                center_mm = int(depth_mm[h // 2, w // 2])
                vis = colorize_depth(depth_mm, args.max_depth)
                interp = cv2.INTER_NEAREST
            else:
                center_mm = None
                vis = raw
                interp = cv2.INTER_LINEAR

            h, w = vis.shape[:2]
            if w != args.width or h != args.height:
                vis = cv2.resize(vis, (args.width, args.height), interpolation=interp)

            cv2.putText(vis, f"FPS: {fps_display:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            if center_mm is not None:
                etiqueta = f"Centro: {center_mm} mm" if center_mm > 0 else "Centro: sin dato"
                cv2.putText(vis, etiqueta, (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow(f"Kinova {args.stream.upper()} - {args.ip}", vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("[INFO] Saliendo...")
                break
            elif key == ord('s'):
                capture_count += 1
                filename = f"kinova_capture_{capture_count}.png"
                cv2.imwrite(filename, vis)
                print(f"[INFO] Captura guardada: {filename}")
                if is_depth:
                    raw_name = f"kinova_capture_{capture_count}_raw16.png"
                    cv2.imwrite(raw_name, raw)
                    print(f"[INFO] Profundidad métrica (16 bits, mm): {raw_name}")

    except KeyboardInterrupt:
        print("\n[INFO] Interrumpido por el usuario")
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
