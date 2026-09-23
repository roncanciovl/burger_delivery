# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""
Lógica pura del nodo de razonamiento espacial con Gemini (sin ROS ni red).

Tres pasos, cada uno probado con ``colcon test``:

1. **Interpretar la respuesta** de Gemini Robotics-ER: un arreglo JSON de
   ``{"point": [y, x], "label": ...}`` con coordenadas normalizadas a 0-1000, a veces
   envuelto en un bloque ```json. Ojo al orden: **[y, x]**.
2. **Leer la profundidad** alrededor del píxel con una mediana que ignora ceros y NaN
   (el sensor devuelve 0 donde no mide, y un solo píxel es demasiado ruidoso).
3. **Desproyectar**: ``image_geometry.PinholeCameraModel.projectPixelTo3dRay`` devuelve un
   rayo **unitario**, así que el punto es ``ray * z / ray_z``, no ``ray * z`` (la guía
   de ``EXPERIMENTO_IA_LOCALIZACION_GEMINI.md`` §6 multiplicaba directamente, lo que
   acorta el punto en ``1/cos(ángulo)`` fuera del eje óptico).
"""

import json
import math
import re
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

_JSON_ARRAY = re.compile(r'\[\s*\{.*\}\s*\]', re.S)


def parse_points(text: str) -> List[Dict]:
    """
    Extraer los puntos de la respuesta del modelo.

    :param text: texto devuelto por ``generate_content``.
    :returns: lista de ``{'y': int, 'x': int, 'label': str}`` en la escala 0-1000;
        vacía si la respuesta no contiene un arreglo válido.
    """
    if not text:
        return []
    match = _JSON_ARRAY.search(text)
    if not match:
        return []
    try:
        items = json.loads(match.group(0))
    except json.JSONDecodeError:
        return []
    points = []
    for item in items:
        if not isinstance(item, dict):
            continue
        point = item.get('point')
        if (not isinstance(point, (list, tuple)) or len(point) != 2
                or not all(isinstance(v, (int, float)) for v in point)):
            continue
        y, x = point
        if not (0 <= y <= 1000 and 0 <= x <= 1000):
            continue
        points.append({'y': float(y), 'x': float(x), 'label': str(item.get('label', ''))})
    return points


def normalized_to_pixel(point: Dict, width: int, height: int) -> Tuple[float, float]:
    """Convertir un punto [y, x] en 0-1000 a píxeles (u, v) de una imagen ``width × height``."""
    return point['x'] / 1000.0 * width, point['y'] / 1000.0 * height


def scale_pixel(u: float, v: float, from_size: Tuple[int, int],
                to_size: Tuple[int, int]) -> Tuple[float, float]:
    """Llevar un píxel de una imagen de tamaño ``(w, h)`` a otra, por escala."""
    return u * to_size[0] / from_size[0], v * to_size[1] / from_size[1]


def depth_at(depth: np.ndarray, u: float, v: float, window: int = 5,
             encoding: str = '16UC1') -> Optional[float]:
    """
    Profundidad en metros alrededor de (u, v), por mediana de una ventana.

    :param depth: imagen de profundidad (``16UC1`` en mm o ``32FC1`` en m).
    :param window: lado de la ventana cuadrada, en píxeles.
    :returns: la mediana de los valores válidos, o ``None`` si no hay ninguno.
    """
    rows, cols = depth.shape[:2]
    cu, cv = int(round(u)), int(round(v))
    if not (0 <= cu < cols and 0 <= cv < rows):
        return None
    half = max(0, window // 2)
    patch = depth[max(0, cv - half):cv + half + 1, max(0, cu - half):cu + half + 1]
    values = patch.astype(np.float64).ravel()
    values = values[np.isfinite(values) & (values > 0)]
    if values.size == 0:
        return None
    z = float(np.median(values))
    return z / 1000.0 if encoding in ('16UC1', 'mono16') else z


def scale_ray_to_depth(ray: Sequence[float], z: float) -> Tuple[float, float, float]:
    """Punto 3D sobre el rayo ``ray`` (de norma cualquiera) cuya coordenada z vale ``z``."""
    if ray[2] <= 0:
        raise ValueError('el rayo no apunta hacia delante de la cámara')
    k = z / ray[2]
    return ray[0] * k, ray[1] * k, z


def deproject(u: float, v: float, z: float, fx: float, fy: float,
              cx: float, cy: float) -> Tuple[float, float, float]:
    """Desproyectar sin distorsión (igual a PinholeCameraModel + scale_ray_to_depth)."""
    return (u - cx) * z / fx, (v - cy) * z / fy, z


def ray_from_intrinsics(u: float, v: float, fx: float, fy: float,
                        cx: float, cy: float) -> Tuple[float, float, float]:
    """Rayo unitario por (u, v), como ``projectPixelTo3dRay`` de image_geometry."""
    x, y = (u - cx) / fx, (v - cy) / fy
    norm = math.sqrt(x * x + y * y + 1.0)
    return x / norm, y / norm, 1.0 / norm
