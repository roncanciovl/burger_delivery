# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
r"""
Benchmark cuantitativo: localización 3D de Gemini frente a AprilTag (TODO.md §1).

Montaje: un AprilTag 36h11 de tamaño conocido pegado al CENTRO de la cara superior de la
caja. En cada ensayo se toma UNA imagen y, sobre esa misma imagen:

* **referencia**: el centro del tag en el marco óptico de la cámara por PnP
  (``tag_geometry.tag_center_in_camera``);
* **Gemini**: el punto que señala el modelo, desproyectado con la profundidad por el mismo
  código que usa el nodo (``deproject_pixel``).

Se registran el error euclidiano 3D (mm), el error en píxeles, la latencia de inferencia
(ms) y los bytes enviados. Al final escribe ``ensayos.csv``, ``resumen.json``, las imágenes
anotadas y, si hay matplotlib, ``figura_benchmark.png``.

Uso (con la cámara publicando y GEMINI_API_KEY en el entorno)::

    ros2 run burger_perception benchmark_gemini_apriltag --tag-id 7 --tag-size 0.05 \\
        --ensayos 10 --interactivo --salida bench_gemini_$(date +%Y%m%d)

Con ``--interactivo`` pide mover la caja entre ensayos; sin él, repite la misma escena
(mide repetibilidad y latencia).
"""

import argparse
import csv
import json
import os
import sys
import threading
import time

from burger_perception.benchmark_stats import error_3d_mm, summarize
from burger_perception.gemini_client import build_prompt, DEFAULT_MODEL, GeminiPointer
from burger_perception.gemini_spatial_reasoning_node import deproject_pixel
from burger_perception.tag_geometry import make_detector, tag_center_in_camera
from burger_perception.vlm_geometry import normalized_to_pixel
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class Captura:
    """Suscripciones a imagen, camera_info y profundidad, con contador de bytes recibidos."""

    def __init__(self, node, args):
        """Crear las suscripciones del benchmark."""
        self.lock = threading.Lock()
        self.image = self.info = self.depth = None
        self.bytes_imagen = 0
        self.t0 = time.monotonic()
        node.create_subscription(CompressedImage, args.image_topic, self._img,
                                 qos_profile_sensor_data)
        node.create_subscription(CameraInfo, args.camera_info_topic, self._info, 10)
        if args.depth_mode != 'plane':
            node.create_subscription(Image, args.depth_topic, self._depth,
                                     qos_profile_sensor_data)

    def _img(self, msg):
        with self.lock:
            self.image = msg
            self.bytes_imagen += len(msg.data)

    def _info(self, msg):
        with self.lock:
            self.info = msg

    def _depth(self, msg):
        with self.lock:
            self.depth = msg

    def instantanea(self, despues_de=None, timeout=10.0):
        """Devolver (image, info, depth) con una imagen posterior a ``despues_de``."""
        limite = time.monotonic() + timeout
        while time.monotonic() < limite:
            with self.lock:
                img, info, depth = self.image, self.info, self.depth
            if img is not None and info is not None and (
                    despues_de is None or _stamp(img) > despues_de):
                return img, info, depth
            time.sleep(0.05)
        raise TimeoutError('no llegan imágenes nuevas con camera_info')

    def ancho_de_banda_bps(self):
        """Bytes/s medios del tópico de imagen comprimida desde el arranque."""
        with self.lock:
            return self.bytes_imagen / max(1e-6, time.monotonic() - self.t0)


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def _jpeg_para_modelo(bgr, lado_max, calidad):
    escala = min(1.0, lado_max / max(bgr.shape[:2]))
    if escala < 1.0:
        bgr = cv2.resize(bgr, None, fx=escala, fy=escala, interpolation=cv2.INTER_AREA)
    ok, datos = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, calidad])
    if not ok:
        raise RuntimeError('no se pudo codificar el JPEG')
    return datos.tobytes()


def ensayo(i, captura, puntero, prompt, detectar, args, anterior):
    """Ejecutar un ensayo y devolver (fila, stamp de la imagen usada)."""
    image, info, depth = captura.instantanea(anterior)
    bgr = cv2.imdecode(np.frombuffer(image.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    alto, ancho = bgr.shape[:2]
    fila = {'ensayo': i, 'stamp': _stamp(image), 'ok_tag': False, 'ok_gemini': False}

    gris = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    esquinas, ids, _ = detectar(gris)
    vistos = [] if ids is None else [int(x) for x in ids.ravel()]
    k = np.array(info.k, dtype=np.float64).reshape(3, 3)
    dist = np.array(info.d, dtype=np.float64) if len(info.d) else np.zeros(5)
    ref = None
    if args.tag_id in vistos:
        ref, ref_px = tag_center_in_camera(
            esquinas[vistos.index(args.tag_id)].reshape(4, 2), args.tag_size, k, dist)
        fila.update({'ok_tag': True, 'tag_x': ref[0], 'tag_y': ref[1], 'tag_z': ref[2],
                     'tag_u': ref_px[0], 'tag_v': ref_px[1]})

    jpeg = _jpeg_para_modelo(bgr, args.lado_max, args.calidad_jpeg)
    puntos, latencia, _ = puntero.point(jpeg, prompt)
    fila.update({'latencia_ms': latencia, 'jpeg_bytes': len(jpeg)})
    gem = None
    if puntos:
        u, v = normalized_to_pixel(puntos[0], ancho, alto)
        gem, _ = deproject_pixel(u, v, (ancho, alto), info, depth, None, args.depth_mode,
                                 args.ventana, args.distancia_plano)
        fila.update({'gem_u': u, 'gem_v': v})
        if gem is not None:
            fila.update({'ok_gemini': True, 'gem_x': gem[0], 'gem_y': gem[1], 'gem_z': gem[2]})

    if ref is not None and gem is not None:
        fila['error_3d_mm'] = error_3d_mm(ref, gem)
        fila['error_px'] = float(np.hypot(fila['gem_u'] - fila['tag_u'],
                                          fila['gem_v'] - fila['tag_v']))

    anotada = bgr.copy()
    if ref is not None:
        cv2.circle(anotada, (int(fila['tag_u']), int(fila['tag_v'])), 12, (0, 200, 0), 3)
    if 'gem_u' in fila:
        cv2.drawMarker(anotada, (int(fila['gem_u']), int(fila['gem_v'])), (0, 0, 255),
                       cv2.MARKER_CROSS, 30, 3)
    cv2.imwrite(os.path.join(args.salida, f'ensayo_{i:02d}.jpg'), anotada)
    return fila, _stamp(image)


def _figura(filas, ruta):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    idx = [f['ensayo'] for f in filas]
    fig, ejes = plt.subplots(1, 3, figsize=(13, 3.8))
    ejes[0].bar(idx, [f.get('error_3d_mm', np.nan) for f in filas], color='#4c72b0')
    ejes[0].set(title='Error 3D Gemini vs AprilTag', xlabel='ensayo', ylabel='mm')
    ejes[1].bar(idx, [f.get('latencia_ms', np.nan) for f in filas], color='#dd8452')
    ejes[1].set(title='Latencia de inferencia', xlabel='ensayo', ylabel='ms')
    errores = [f['error_3d_mm'] for f in filas if 'error_3d_mm' in f]
    if errores:
        ejes[2].boxplot(errores)
    ejes[2].set(title='Distribución del error 3D', ylabel='mm', xticks=[])
    fig.tight_layout()
    fig.savefig(ruta, dpi=150)
    return True


def main(argv=None):
    """Punto de entrada del benchmark."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--tag-id', type=int, required=True)
    parser.add_argument('--tag-size', type=float, required=True, help='lado del tag, m')
    parser.add_argument('--ensayos', type=int, default=10)
    parser.add_argument('--interactivo', action='store_true')
    parser.add_argument('--salida', default=time.strftime('bench_gemini_%Y%m%d_%H%M%S'))
    parser.add_argument('--image-topic', default='/camera/color/image_raw/compressed')
    parser.add_argument('--camera-info-topic', default='/camera/color/camera_info')
    parser.add_argument('--depth-topic', default='/camera/depth_registered/image_rect')
    parser.add_argument('--depth-mode', choices=['registered', 'plane'], default='registered')
    parser.add_argument('--distancia-plano', type=float, default=0.5)
    parser.add_argument('--ventana', type=int, default=7)
    parser.add_argument('--target', default='cardboard burger box')
    parser.add_argument('--context', default='It must be resting on the table.')
    parser.add_argument('--modelo', default=DEFAULT_MODEL)
    parser.add_argument('--lado-max', type=int, default=1024)
    parser.add_argument('--calidad-jpeg', type=int, default=85)
    args = parser.parse_args(remove_ros_args(argv or sys.argv)[1:])

    os.makedirs(args.salida, exist_ok=True)
    puntero = GeminiPointer(args.modelo)
    prompt = build_prompt(args.target, args.context)
    detectar = make_detector()

    rclpy.init()
    node = rclpy.create_node('benchmark_gemini_apriltag')
    captura = Captura(node, args)
    hilo = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    hilo.start()

    filas, anterior = [], None
    try:
        for i in range(1, args.ensayos + 1):
            if args.interactivo:
                input(f'[{i}/{args.ensayos}] Coloca la caja y pulsa Enter... ')
            fila, anterior = ensayo(i, captura, puntero, prompt, detectar, args, anterior)
            filas.append(fila)
            print(f"  ensayo {i}: tag={'sí' if fila['ok_tag'] else 'NO'} "
                  f"gemini={'sí' if fila['ok_gemini'] else 'NO'} "
                  f"error={fila.get('error_3d_mm', float('nan')):.1f} mm "
                  f"latencia={fila['latencia_ms']:.0f} ms")
    finally:
        banda = captura.ancho_de_banda_bps()
        node.destroy_node()
        rclpy.try_shutdown()

    columnas = sorted({c for f in filas for c in f}, key=lambda c: (c != 'ensayo', c))
    with open(os.path.join(args.salida, 'ensayos.csv'), 'w', newline='') as f:
        escritor = csv.DictWriter(f, fieldnames=columnas)
        escritor.writeheader()
        escritor.writerows(filas)
    duracion = max(1e-6, filas[-1]['stamp'] - filas[0]['stamp']) if len(filas) > 1 else 1.0
    resumen = {
        'modelo': args.modelo, 'depth_mode': args.depth_mode, 'tag_id': args.tag_id,
        'tag_size_m': args.tag_size, 'ensayos': len(filas),
        'exito_tag': sum(f['ok_tag'] for f in filas),
        'exito_gemini': sum(f['ok_gemini'] for f in filas),
        'error_3d_mm': summarize([f.get('error_3d_mm') for f in filas]),
        'error_px': summarize([f.get('error_px') for f in filas]),
        'latencia_ms': summarize([f.get('latencia_ms') for f in filas]),
        'jpeg_bytes_medio': summarize([f.get('jpeg_bytes') for f in filas]).get('media'),
        'camara_comprimida_Bps': banda,
        'subida_gemini_Bps': sum(f['jpeg_bytes'] for f in filas) / duracion,
    }
    with open(os.path.join(args.salida, 'resumen.json'), 'w') as f:
        json.dump(resumen, f, indent=2)
    figura = _figura(filas, os.path.join(args.salida, 'figura_benchmark.png'))
    e = resumen['error_3d_mm']
    print(f"\nError 3D: n={e.get('n', 0)} media={e.get('media', float('nan')):.1f} mm "
          f"RMSE={e.get('rmse', float('nan')):.1f} mm p95={e.get('p95', float('nan')):.1f} mm")
    print(f"Latencia: media={resumen['latencia_ms'].get('media', float('nan')):.0f} ms")
    print(f'Resultados en {args.salida}/' + (' (con figura)' if figura else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
