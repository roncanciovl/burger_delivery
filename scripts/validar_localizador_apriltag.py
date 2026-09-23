#!/usr/bin/env python3
# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""
Valida el localizador AprilTag con la cámara real del Kinova (TODO.md §4.1).

Dos modos, pensados para usarse en este orden durante la prueba en el laboratorio:

``imagen``  Toma UNA imagen del tópico comprimido, detecta los tags 36h11 y reporta, por
            cada uno, su ID, su centro y su lado aparente en píxeles. Guarda la imagen
            anotada. Sirve para confirmar qué IDs hay en la mesa, que se ven desde la pose
            de observación y que tienen resolución suficiente (≥ 30 px de lado).

``pose``    Con el localizador corriendo, registra N mensajes de ``/<ns>/pose2d`` con el
            tag del carrito en una posición MEDIDA con cinta (``--esperado x y theta_deg``)
            y calcula media, desviación y error. Añade una fila a un CSV, una por posición,
            que es la tabla que pide ``vision_setup/VALIDACION_LOCALIZADOR_REAL.md``.

Uso::

    python3 scripts/validar_localizador_apriltag.py imagen --salida validacion/
    python3 scripts/validar_localizador_apriltag.py pose --esperado 0.30 0.15 0 \\
        --etiqueta P1_mesa --muestras 100 --csv validacion/poses.csv

La lógica de cálculo (``resumir_poses``, ``diagnosticar_tags``) no depende de ROS y se
prueba sin robot.
"""

import argparse
import csv
import math
import os
import statistics
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

LADO_MINIMO_PX = 30.0


def _dif_angular(a: float, b: float) -> float:
    """Diferencia ``a - b`` normalizada a [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def resumir_poses(poses: Sequence[Tuple[float, float, float]],
                  esperado: Optional[Tuple[float, float, float]] = None) -> Dict[str, float]:
    """
    Resumir una serie de poses 2D (x, y en m; theta en rad).

    La media de theta es circular, para que una serie alrededor de ±180° no promedie 0°.

    :returns: ``n``, medias y desviaciones de x, y, theta (theta en grados) y, si se da
        ``esperado``, el error de posición (mm) y de orientación (grados) de la media.
    """
    if not poses:
        raise ValueError('no se recibió ninguna pose')
    xs = [p[0] for p in poses]
    ys = [p[1] for p in poses]
    ts = [p[2] for p in poses]
    t_media = math.atan2(sum(math.sin(t) for t in ts), sum(math.cos(t) for t in ts))
    desv_t = [math.degrees(_dif_angular(t, t_media)) for t in ts]
    resumen = {
        'n': len(poses),
        'x_media_m': statistics.fmean(xs),
        'y_media_m': statistics.fmean(ys),
        'theta_media_deg': math.degrees(t_media),
        'x_std_mm': 1000.0 * statistics.pstdev(xs),
        'y_std_mm': 1000.0 * statistics.pstdev(ys),
        'theta_std_deg': math.sqrt(statistics.fmean(d * d for d in desv_t)),
    }
    if esperado is not None:
        ex, ey, et = esperado
        resumen['error_pos_mm'] = 1000.0 * math.hypot(resumen['x_media_m'] - ex,
                                                      resumen['y_media_m'] - ey)
        resumen['error_theta_deg'] = math.degrees(_dif_angular(t_media, et))
    return resumen


def diagnosticar_tags(corners, ids) -> List[Dict[str, float]]:
    """
    Describir los tags detectados: ID, centro (px), lado medio (px) y si es suficiente.

    :param corners: esquinas tal como las devuelve ``detectMarkers``.
    :param ids: IDs tal como los devuelve ``detectMarkers`` (o ``None``).
    """
    if ids is None:
        return []
    salida = []
    for esquinas, tag_id in zip(corners, [int(i) for i in ids.ravel()]):
        p = esquinas.reshape(4, 2)
        lados = [math.dist(p[k], p[(k + 1) % 4]) for k in range(4)]
        lado = sum(lados) / 4.0
        salida.append({
            'id': tag_id,
            'u': float(p[:, 0].mean()),
            'v': float(p[:, 1].mean()),
            'lado_px': lado,
            # Un cuadrilátero muy distinto de un cuadrado delata inclinación fuerte.
            'asimetria': (max(lados) - min(lados)) / lado if lado else 0.0,
            'suficiente': lado >= LADO_MINIMO_PX,
        })
    return sorted(salida, key=lambda t: t['id'])


def _esperar_mensajes(node, tipo, topico, n, timeout_s, qos):
    """Suscribirse a ``topico`` y devolver hasta ``n`` mensajes o lo que llegue en plazo."""
    import rclpy
    recibidos = []
    node.create_subscription(tipo, topico, recibidos.append, qos)
    limite = time.monotonic() + timeout_s
    while rclpy.ok() and len(recibidos) < n and time.monotonic() < limite:
        rclpy.spin_once(node, timeout_sec=0.1)
    return recibidos


def modo_imagen(args) -> int:
    import cv2
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage

    rclpy.init()
    node = rclpy.create_node('validar_localizador_imagen')
    try:
        msgs = _esperar_mensajes(node, CompressedImage, args.image_topic, 1, args.timeout,
                                 qos_profile_sensor_data)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if not msgs:
        print(f'✗ no llegó ninguna imagen de {args.image_topic} en {args.timeout:.0f} s. '
              '¿La anfitriona publica la cámara? (ros2 topic hz del tópico)')
        return 2
    gris = cv2.imdecode(np.frombuffer(msgs[0].data, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    diccionario = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        corners, ids, _ = cv2.aruco.ArucoDetector(
            diccionario, cv2.aruco.DetectorParameters()).detectMarkers(gris)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            gris, diccionario, parameters=cv2.aruco.DetectorParameters_create())
    tags = diagnosticar_tags(corners, ids)
    print(f'Imagen {gris.shape[1]}×{gris.shape[0]} de {args.image_topic}')
    if not tags:
        print('✗ ningún tag 36h11 visible. Revisa la pose de observación, el enfoque y '
              'que los tags sean de la familia 36h11.')
    for t in tags:
        estado = 'OK' if t['suficiente'] else f'PEQUEÑO (< {LADO_MINIMO_PX:.0f} px)'
        print(f"  ID {t['id']:3d}  centro ({t['u']:7.1f}, {t['v']:7.1f}) px  "
              f"lado {t['lado_px']:6.1f} px  asimetría {100 * t['asimetria']:4.1f} %  {estado}")
    os.makedirs(args.salida, exist_ok=True)
    anotada = cv2.cvtColor(gris, cv2.COLOR_GRAY2BGR)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(anotada, corners, ids)
    ruta = os.path.join(args.salida, time.strftime('tags_%Y%m%d_%H%M%S.jpg'))
    cv2.imwrite(ruta, anotada)
    print(f'Imagen anotada: {ruta}')
    return 0 if tags else 1


def modo_pose(args) -> int:
    import rclpy
    from geometry_msgs.msg import Pose2D

    topico = f'/{args.robot_namespace}/pose2d'
    rclpy.init()
    node = rclpy.create_node('validar_localizador_pose')
    inicio = time.monotonic()
    try:
        msgs = _esperar_mensajes(node, Pose2D, topico, args.muestras, args.timeout, 10)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    duracion = time.monotonic() - inicio
    if not msgs:
        print(f'✗ no llegó ninguna pose en {topico}. ¿Corre el localizador y ve los dos tags?')
        return 2
    esperado = None
    if args.esperado:
        ex, ey, et = args.esperado
        esperado = (ex, ey, math.radians(et))
    r = resumir_poses([(m.x, m.y, m.theta) for m in msgs], esperado)
    print(f"{r['n']} poses en {duracion:.1f} s ({r['n'] / duracion:.1f} Hz) de {topico}")
    print(f"  x = {r['x_media_m']:.4f} m ± {r['x_std_mm']:.1f} mm")
    print(f"  y = {r['y_media_m']:.4f} m ± {r['y_std_mm']:.1f} mm")
    print(f"  θ = {r['theta_media_deg']:.2f}° ± {r['theta_std_deg']:.2f}°")
    if esperado is not None:
        print(f"  error de posición {r['error_pos_mm']:.1f} mm, "
              f"de orientación {r['error_theta_deg']:+.2f}°")
    if len(msgs) < args.muestras:
        print(f'  ⚠ sólo {len(msgs)} de {args.muestras} poses: algún tag dejó de verse')
    if args.csv:
        os.makedirs(os.path.dirname(os.path.abspath(args.csv)), exist_ok=True)
        nuevo = not os.path.exists(args.csv)
        fila = {'etiqueta': args.etiqueta, 'fecha': time.strftime('%Y-%m-%dT%H:%M:%S'),
                'hz': r['n'] / duracion,
                'esperado_x_m': args.esperado[0] if args.esperado else '',
                'esperado_y_m': args.esperado[1] if args.esperado else '',
                'esperado_theta_deg': args.esperado[2] if args.esperado else '',
                **{k: (round(v, 5) if isinstance(v, float) else v) for k, v in r.items()}}
        with open(args.csv, 'a', newline='', encoding='utf-8') as f:
            escritor = csv.DictWriter(f, fieldnames=list(fila))
            if nuevo:
                escritor.writeheader()
            escritor.writerow(fila)
        print(f'  fila añadida a {args.csv}')
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = parser.add_subparsers(dest='modo', required=True)
    p_img = sub.add_parser('imagen', help='detectar tags en una imagen de la cámara')
    p_img.add_argument('--image-topic', default='/camera/color/image_raw/compressed')
    p_img.add_argument('--salida', default='validacion_apriltag')
    p_img.add_argument('--timeout', type=float, default=10.0)
    p_pose = sub.add_parser('pose', help='estadística de /<ns>/pose2d en una posición')
    p_pose.add_argument('--robot-namespace', default='burger_car_01')
    p_pose.add_argument('--muestras', type=int, default=100)
    p_pose.add_argument('--esperado', type=float, nargs=3, metavar=('X_M', 'Y_M', 'THETA_DEG'))
    p_pose.add_argument('--etiqueta', default='sin_etiqueta')
    p_pose.add_argument('--csv', default='')
    p_pose.add_argument('--timeout', type=float, default=30.0)
    args = parser.parse_args(argv)
    return modo_imagen(args) if args.modo == 'imagen' else modo_pose(args)


if __name__ == '__main__':
    sys.exit(main())
