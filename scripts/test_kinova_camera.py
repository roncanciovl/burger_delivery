#!/usr/bin/env python3
"""
Kinova Gen3 - Visor de cámara en tiempo real
Conexión directa por RTSP sin pasar por ROS.
Uso:  python3 kinova_camera.py [--ip 192.168.1.10] [--stream color|depth]
"""
import cv2
import argparse
import sys
import time
import os


def try_gstreamer(rtsp_url, stream_type, width, height):
    """Intentar conexión con GStreamer pipeline."""
    if stream_type == 'color':
        pipeline = (
            f"rtspsrc location={rtsp_url} latency=0 buffer-mode=auto "
            f"! rtph264depay "
            f"! avdec_h264 "
            f"! videoconvert "
            f"! videoscale "
            f"! video/x-raw,format=BGR,width={width},height={height} "
            f"! appsink drop=true max-buffers=1 sync=false"
        )
    else:
        pipeline = (
            f"rtspsrc location={rtsp_url} latency=0 buffer-mode=auto "
            f"! rtpgstdepay "
            f"! videoconvert "
            f"! videoscale "
            f"! video/x-raw,format=BGR,width={width},height={height} "
            f"! appsink drop=true max-buffers=1 sync=false"
        )

    print(f"[INFO] Intentando GStreamer pipeline...")
    print(f"[DEBUG] Pipeline: {pipeline}")
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        # Intentar leer un frame para confirmar que funciona
        ret, _ = cap.read()
        if ret:
            print("[INFO] ✓ Conectado con GStreamer")
            return cap
        else:
            cap.release()
    print("[WARN] GStreamer pipeline no funcionó")
    return None


def try_ffmpeg(rtsp_url, width, height):
    """Intentar conexión con FFMPEG (TCP transport)."""
    print("[INFO] Intentando FFMPEG con TCP...")

    # Configurar opciones FFMPEG para baja latencia
    os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = (
        'rtsp_transport;tcp|'
        'fflags;nobuffer|'
        'flags;low_delay|'
        'max_delay;0|'
        'analyzeduration;100000|'
        'probesize;100000'
    )

    cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ret, _ = cap.read()
        if ret:
            print("[INFO] ✓ Conectado con FFMPEG TCP")
            return cap
        else:
            cap.release()

    print("[WARN] FFMPEG TCP no funcionó")
    return None


def try_ffmpeg_udp(rtsp_url, width, height):
    """Intentar conexión con FFMPEG (UDP transport)."""
    print("[INFO] Intentando FFMPEG con UDP...")

    os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = (
        'rtsp_transport;udp|'
        'fflags;nobuffer|'
        'flags;low_delay|'
        'max_delay;0|'
        'analyzeduration;100000|'
        'probesize;100000'
    )

    cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ret, _ = cap.read()
        if ret:
            print("[INFO] ✓ Conectado con FFMPEG UDP")
            return cap
        else:
            cap.release()

    print("[WARN] FFMPEG UDP no funcionó")
    return None


def try_default(rtsp_url):
    """Intentar conexión con backend por defecto de OpenCV."""
    print("[INFO] Intentando backend por defecto...")
    cap = cv2.VideoCapture(rtsp_url)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ret, _ = cap.read()
        if ret:
            print("[INFO] ✓ Conectado con backend por defecto")
            return cap
        else:
            cap.release()

    print("[WARN] Backend por defecto no funcionó")
    return None


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
    args = parser.parse_args()

    rtsp_url = f"rtsp://{args.ip}/{args.stream}"
    print(f"[INFO] Conectando a: {rtsp_url}")
    print(f"[INFO] Resolución de visualización: {args.width}x{args.height}")
    print(f"[INFO] Presiona 'q' para salir, 's' para guardar captura")
    print()

    # Intentar múltiples métodos de conexión (Priorizamos FFMPEG porque GStreamer tiende a colgarse sin timeout duro)
    cap = None
    for attempt_fn in [
        lambda: try_ffmpeg(rtsp_url, args.width, args.height),
        lambda: try_ffmpeg_udp(rtsp_url, args.width, args.height),
        lambda: try_default(rtsp_url),
        lambda: try_gstreamer(rtsp_url, args.stream, args.width, args.height),
    ]:
        cap = attempt_fn()
        if cap is not None:
            break

    if cap is None:
        print(f"\n[ERROR] No se pudo conectar a {rtsp_url}")
        print("[ERROR] Verifica que:")
        print("  1. El robot esté encendido")
        print(f"  2. La IP {args.ip} sea correcta")
        print("  3. La cámara esté habilitada en la configuración del robot")
        sys.exit(1)

    print(f"\n[INFO] ¡Mostrando imagen! (Presiona 'q' para salir)")

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

            # Calcular FPS real
            frame_count += 1
            elapsed = time.time() - fps_start
            if elapsed >= 1.0:
                fps_display = frame_count / elapsed
                frame_count = 0
                fps_start = time.time()

            # Redimensionar si viene en otra resolución
            h, w = frame.shape[:2]
            if w != args.width or h != args.height:
                frame = cv2.resize(frame, (args.width, args.height))

            # Mostrar FPS en la imagen
            cv2.putText(frame, f"FPS: {fps_display:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            cv2.imshow(f"Kinova {args.stream.upper()} - {args.ip}", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("[INFO] Saliendo...")
                break
            elif key == ord('s'):
                capture_count += 1
                filename = f"kinova_capture_{capture_count}.png"
                cv2.imwrite(filename, frame)
                print(f"[INFO] Captura guardada: {filename}")

    except KeyboardInterrupt:
        print("\n[INFO] Interrumpido por el usuario")
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
