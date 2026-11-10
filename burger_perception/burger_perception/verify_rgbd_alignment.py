# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
r"""
Verificar la alineación RGB-profundidad del Kinova antes de usar Gemini.

Requiere la cámara y el brazo quietos, con un objeto de bordes rectos sobre la mesa y a la
vista de ambas cámaras. No mueve el robot ni llama a ningún servicio externo.

    ros2 launch burger_perception camera.launch.py
    ros2 run burger_perception verify_rgbd_alignment --ros-args -p output_dir:=/tmp/rgbd

Escribe ``alineacion_rgbd.json`` y ``alineacion_rgbd.png`` (silueta con los extrínsecos del
sistema en rojo y con los estimados en verde). Código de salida: 0 alineada, 1 desalineada,
2 no concluyente o sin datos.
"""

import json
import os
import sys
import time

from burger_perception.rgbd_alignment import (
    ALIGNED,
    direction_spread,
    edge_distance_field,
    evaluate_alignment,
    INCONCLUSIVE,
    MISALIGNED,
    project_points,
    search_shift,
    silhouette_points,
)
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

# camera_link -> camera_depth_frame en cada fuente de Kinova (el color coincide con camera_link).
LAUNCH_T = (-0.0195, -0.005, 0.0)   # kinova_vision.launch.py: lo que usa el registro
URDF_T = (-0.0275, -0.00961, 0.0)   # macro oficial gen3_macro.xacro


def intrinsics(info):
    """Devolver (fx, fy, cx, cy) de la matriz P, la que usan image_geometry y el registro."""
    return float(info.p[0]), float(info.p[5]), float(info.p[2]), float(info.p[6])


def to_meters(msg):
    """Convertir un mensaje Image de profundidad (16UC1 en mm o 32FC1 en m) a metros."""
    dtype, scale = {'16UC1': (np.uint16, 0.001), 'mono16': (np.uint16, 0.001),
                    '32FC1': (np.float32, 1.0)}[msg.encoding]
    array = np.frombuffer(msg.data, dtype=dtype)
    array = array.reshape(msg.height, msg.step // np.dtype(dtype).itemsize)[:, :msg.width]
    return array.astype(np.float32) * scale


def median_depth(frames, min_valid_fraction=0.5):
    """Mediana por píxel ignorando ceros; 0 donde hubo medida en menos de la mitad."""
    stack = np.stack(frames).astype(np.float32)
    enough = (stack > 0).mean(axis=0) >= min_valid_fraction
    stack[stack <= 0] = np.nan
    result = np.zeros(stack.shape[1:], dtype=np.float32)
    result[enough] = np.nanmedian(stack[:, enough], axis=0)
    return result


class AlignmentProbe(Node):
    """Reunir imágenes de la cámara para analizarlas una sola vez."""

    def __init__(self):
        """Declarar parámetros y suscripciones."""
        super().__init__('verify_rgbd_alignment')
        p = self.declare_parameter
        p('color_topic', '/camera/color/image_raw/compressed')
        p('color_info_topic', '/camera/color/camera_info')
        p('depth_topic', '/camera/depth/image_raw')
        p('depth_info_topic', '/camera/depth/camera_info')
        p('registered_topic', '/camera/depth_registered/image_rect')
        p('frames', 30)
        p('timeout_s', 30.0)
        p('tol_mm', 3.0)
        p('max_motion', 6.0)
        p('output_dir', '.')

        g = self.get_parameter
        self.target = g('frames').value
        self.first_color = None
        self.last_color = None
        self.color_count = 0
        self.depth_frames = []
        self.registered = None
        self.color_info = None
        self.depth_info = None
        self.create_subscription(CompressedImage, g('color_topic').value, self._on_color,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, g('depth_topic').value, self._on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, g('registered_topic').value, self._on_registered,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(CameraInfo, g('color_info_topic').value, self._on_color_info, 10)
        self.create_subscription(CameraInfo, g('depth_info_topic').value, self._on_depth_info, 10)

    def _on_color(self, msg):
        self.first_color = self.first_color or msg
        self.last_color = msg
        self.color_count += 1

    def _on_depth(self, msg):
        if len(self.depth_frames) < self.target:
            self.depth_frames.append(to_meters(msg))

    def _on_registered(self, msg):
        self.registered = msg

    def _on_color_info(self, msg):
        self.color_info = msg

    def _on_depth_info(self, msg):
        self.depth_info = msg

    def ready(self):
        """Indicar si ya hay datos suficientes."""
        return (len(self.depth_frames) >= self.target and self.color_count >= self.target
                and self.color_info is not None and self.depth_info is not None)


def decode_gray(msg):
    """Decodificar una imagen comprimida a escala de grises."""
    return cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)


def analyze(probe):
    """Analizar lo capturado y devolver (reporte, imagen de color, proyecciones)."""
    gray = decode_gray(probe.last_color)
    motion = float(np.mean(cv2.absdiff(decode_gray(probe.first_color), gray)))
    depth = median_depth(probe.depth_frames)
    u, v, z = silhouette_points(depth)
    color_k, depth_k = intrinsics(probe.color_info), intrinsics(probe.depth_info)
    field = edge_distance_field(gray)
    tol = probe.get_parameter('tol_mm').value
    report = evaluate_alignment(field, u, v, z, depth_k, color_k,
                                {'launch': LAUNCH_T, 'urdf': URDF_T}, 'launch', tol)
    report.update({'motion': motion, 'frames': len(probe.depth_frames),
                   'color_size': [gray.shape[1], gray.shape[0]],
                   'color_P': list(color_k), 'depth_P': list(depth_k)})

    limit = probe.get_parameter('max_motion').value
    if motion > limit:
        report['verdict'] = INCONCLUSIVE
        report['reason'] = (f'la cámara o la escena se movió durante la captura '
                            f'(diferencia {motion:.1f}, máximo {limit:.1f})')

    if probe.registered is not None:
        registered = to_meters(probe.registered)
        if registered.shape == gray.shape:
            ru, rv, _ = silhouette_points(registered, max_gap_px=100, min_component_px=40)
            if ru.size >= 200:
                report['registered_shift_px'] = search_shift(field, ru, rv)
                report['registered_shift_px']['direction_spread'] = direction_spread(ru, rv)

    best = report.get('best_t_mm')
    projections = {'launch': project_points(u, v, z, depth_k, color_k, LAUNCH_T)}
    if best is not None:
        projections['best'] = project_points(u, v, z, depth_k, color_k,
                                             [c / 1000.0 for c in best])
    return report, cv2.imdecode(np.frombuffer(probe.last_color.data, np.uint8),
                                cv2.IMREAD_COLOR), projections


def save_overlay(path, image, projections):
    """Dibujar la silueta proyectada sobre el color: sistema en rojo y estimada en verde."""
    colors = {'launch': (0, 0, 255), 'best': (0, 220, 0)}
    for name, (u, v) in projections.items():
        for x, y in zip(np.rint(u).astype(int), np.rint(v).astype(int)):
            if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                cv2.circle(image, (int(x), int(y)), 2, colors[name], -1)
    cv2.imwrite(path, image)


def print_report(report):
    """Mostrar un resumen legible del reporte."""
    print(f'\nVEREDICTO: {report["verdict"]}\n  {report["reason"]}')
    print(f'  puntos de silueta: {report["points"]} dentro del color '
          f'({report.get("points_outside_color", 0)} fuera), '
          f'movimiento: {report.get("motion", 0):.1f}')
    if 'best_t_mm' in report:
        print(f'  mejor estimación t (mm): {report["best_t_mm"]}, '
              f'chamfer {report["best_chamfer_px"]:.2f} px, '
              f'contraste {report["contrast"]:.2f}, ambigüedad {report["ambiguity"]:.2f}, '
              f'z mediana {report["median_depth_m"]:.3f} m')
    for name, item in report['hypotheses'].items():
        print(f'  {name:7s} t={item["t_mm"]} mm, chamfer {item["chamfer_px"]:.2f} px, '
              f'error {item["error_mm"]:.1f} mm ({item["error_px_at_median_depth"]:.0f} px)')
    shift = report.get('registered_shift_px')
    if shift:
        print(f'  profundidad registrada: desplazamiento residual ({shift["du"]}, {shift["dv"]}) '
              f'px, chamfer {shift["score"]:.2f} px (en cero: {shift["score_at_zero"]:.2f})')
        if shift['direction_spread'] < 0.2:
            print('    (silueta casi recta: este desplazamiento no es fiable)')


def main(args=None):
    """Capturar, analizar, guardar el reporte y salir con el código del veredicto."""
    rclpy.init(args=args)
    probe = AlignmentProbe()
    deadline = time.monotonic() + probe.get_parameter('timeout_s').value
    while rclpy.ok() and time.monotonic() < deadline and not probe.ready():
        rclpy.spin_once(probe, timeout_sec=0.05)
    if not probe.ready():
        print(f'Sin datos suficientes: {len(probe.depth_frames)} imágenes de profundidad, '
              f'{probe.color_count} de color. ¿Está corriendo camera.launch.py?', file=sys.stderr)
        probe.destroy_node()
        rclpy.try_shutdown()
        sys.exit(2)

    report, image, projections = analyze(probe)
    output_dir = probe.get_parameter('output_dir').value
    os.makedirs(output_dir, exist_ok=True)
    with open(os.path.join(output_dir, 'alineacion_rgbd.json'), 'w', encoding='utf-8') as handle:
        json.dump(report, handle, indent=1, ensure_ascii=False)
    save_overlay(os.path.join(output_dir, 'alineacion_rgbd.png'), image, projections)
    print_report(report)
    probe.destroy_node()
    rclpy.try_shutdown()
    sys.exit({ALIGNED: 0, MISALIGNED: 1}.get(report['verdict'], 2))


if __name__ == '__main__':
    main()
