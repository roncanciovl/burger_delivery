# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""
Lógica pura de la navegación de entrega (sin ROS), probada con ``colcon test``.

Todo es geometría en el plano (SE(2)): el carrito se mueve sobre la mesa o el piso y sólo
importan x, y y el giro alrededor de z.
"""

import math
from typing import Dict, Tuple

Pose2 = Tuple[float, float, float]   # (x, y, yaw)


def normalize_angle(angle: float) -> float:
    """Llevar un ángulo a [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Giro alrededor de z de un cuaternión (ignora la inclinación)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Cuaternión (x, y, z, w) de un giro puro alrededor de z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def compose(a: Pose2, b: Pose2) -> Pose2:
    """Componer transformaciones del plano: ``a ∘ b`` (b expresada en el marco de a)."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return ax + c * bx - s * by, ay + s * bx + c * by, normalize_angle(at + bt)


def inverse(a: Pose2) -> Pose2:
    """Inversa de una transformación del plano."""
    ax, ay, at = a
    c, s = math.cos(at), math.sin(at)
    return -c * ax - s * ay, s * ax - c * ay, normalize_angle(-at)


def map_to_odom(map_to_base: Pose2, odom_to_base: Pose2) -> Pose2:
    """
    Corrección ``map -> odom`` a partir de la localización absoluta del carrito.

    Es lo que haría AMCL, pero con la pose que da el AprilTag: la odometría de ruedas
    mantiene la continuidad y la corrección absorbe su deriva.
    """
    return compose(map_to_base, inverse(odom_to_base))


def pose_error(target: Pose2, current: Pose2) -> Tuple[float, float]:
    """Error de posición (m) y de orientación (rad, con signo) de ``current`` ante ``target``."""
    return (math.hypot(current[0] - target[0], current[1] - target[1]),
            normalize_angle(current[2] - target[2]))


def within_tolerance(target: Pose2, current: Pose2, tol_xy: float, tol_yaw: float) -> bool:
    """Indicar si ``current`` está dentro de las tolerancias de ``target``."""
    error_xy, error_yaw = pose_error(target, current)
    return error_xy <= tol_xy and abs(error_yaw) <= tol_yaw


def parse_slots(raw: Dict) -> Dict[str, Dict]:
    """
    Validar las zonas de entrega leídas del YAML.

    Formato: ``{nombre: {frame, x, y, yaw_deg, tolerance_xy, tolerance_yaw_deg}}``.
    Las tolerancias son opcionales. Lanza ``ValueError`` con el campo que falta.

    :returns: ``{nombre: {'frame', 'pose': (x, y, yaw_rad), 'tol_xy', 'tol_yaw'}}``.
    """
    slots = {}
    for name, spec in (raw or {}).items():
        for key in ('frame', 'x', 'y', 'yaw_deg'):
            if key not in spec:
                raise ValueError(f'slot "{name}": falta "{key}"')
        slots[name] = {
            'frame': str(spec['frame']),
            'pose': (float(spec['x']), float(spec['y']), math.radians(float(spec['yaw_deg']))),
            'tol_xy': float(spec.get('tolerance_xy', 0.0)),
            'tol_yaw': math.radians(float(spec.get('tolerance_yaw_deg', 0.0))),
        }
    return slots


def resolve_tolerance(requested: float, slot_value: float, default: float) -> float:
    """Elegir la tolerancia: la pedida si es > 0, si no la del slot, si no la por defecto."""
    for value in (requested, slot_value):
        if value > 0.0:
            return value
    return default
