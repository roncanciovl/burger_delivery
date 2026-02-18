#!/usr/bin/env python3
"""
test_camera.py
Prueba simple para verificar que la cámara funciona en WSL2.
Muestra el video de la cámara en una ventana con información de FPS.

Uso:
    python3 test_camera.py
    python3 test_camera.py --camera 1    # Para usar otra cámara
    python3 test_camera.py --headless    # Sin ventana, solo verifica captura
"""

import argparse
import sys
import time


def check_camera_devices():
    """Verifica si hay dispositivos de video disponibles."""
    import os
    devices = [f for f in os.listdir('/dev') if f.startswith('video')]
    if devices:
        print(f"✓ Dispositivos de video encontrados: {', '.join(sorted(devices))}")
        return True
    else:
        print("✗ No se encontraron dispositivos /dev/video*")
        print("")
        print("  Posibles soluciones:")
        print("  1. Ejecuta: bash setup_camera_wsl.sh")
        print("  2. Conecta la cámara desde Windows:")
        print("     PowerShell (Admin): usbipd attach --wsl --busid <BUSID>")
        return False


def test_camera(camera_index: int = 0, headless: bool = False):
    """Prueba la captura de la cámara."""
    try:
        import cv2
    except ImportError:
        print("✗ OpenCV no está instalado.")
        print("  Instala con: pip3 install opencv-python")
        sys.exit(1)

    print(f"\nAbriendo cámara {camera_index}...")
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print(f"✗ No se pudo abrir la cámara {camera_index}")
        print(f"  Intenta con otro índice: python3 test_camera.py --camera 1")
        sys.exit(1)

    # Obtener info de la cámara
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps_cam = cap.get(cv2.CAP_PROP_FPS)

    print(f"✓ Cámara abierta exitosamente!")
    print(f"  Resolución: {width}x{height}")
    print(f"  FPS reportados: {fps_cam}")
    print("")

    if headless:
        # Solo verificar que puede capturar frames
        print("Modo headless: capturando 30 frames de prueba...")
        frame_count = 0
        start_time = time.time()
        for _ in range(30):
            ret, frame = cap.read()
            if ret:
                frame_count += 1
        elapsed = time.time() - start_time
        fps_real = frame_count / elapsed if elapsed > 0 else 0
        print(f"✓ Capturados {frame_count}/30 frames en {elapsed:.2f}s ({fps_real:.1f} FPS)")
        cap.release()
        return

    # Modo ventana
    print("Mostrando video en ventana. Presiona 'q' para salir.")
    print("")

    frame_count = 0
    start_time = time.time()
    fps_display = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("✗ Error leyendo frame")
            break

        frame_count += 1
        elapsed = time.time() - start_time
        if elapsed >= 1.0:
            fps_display = frame_count / elapsed
            frame_count = 0
            start_time = time.time()

        # Agregar texto con info
        info_text = f"FPS: {fps_display:.1f} | Res: {width}x{height} | Cam: {camera_index}"
        cv2.putText(frame, info_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, "Presiona 'q' para salir", (10, height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("Test Camara WSL2", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("✓ Cámara cerrada correctamente")


def main():
    parser = argparse.ArgumentParser(description="Prueba de cámara en WSL2")
    parser.add_argument('--camera', type=int, default=0,
                        help='Índice de la cámara (default: 0)')
    parser.add_argument('--headless', action='store_true',
                        help='Modo sin ventana, solo verifica captura')
    args = parser.parse_args()

    print("=" * 60)
    print("  Test de Cámara en WSL2")
    print("=" * 60)
    print("")

    # Verificar dispositivos
    if not check_camera_devices():
        sys.exit(1)

    # Probar cámara
    test_camera(camera_index=args.camera, headless=args.headless)


if __name__ == '__main__':
    main()
