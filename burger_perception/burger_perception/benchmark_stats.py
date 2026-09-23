# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""Estadística del benchmark Gemini frente a AprilTag (puro, sin ROS)."""

import math
import statistics
from typing import Dict, Sequence


def error_3d_mm(a: Sequence[float], b: Sequence[float]) -> float:
    """Distancia euclidiana entre dos puntos en metros, expresada en milímetros."""
    return 1000.0 * math.dist(a, b)


def summarize(values: Sequence[float]) -> Dict[str, float]:
    """
    Resumir una serie: n, media, desviación, mediana, p95, RMSE (respecto a 0) y máximo.

    Se usa para el error 3D (mm), el error en píxeles y la latencia (ms).
    """
    data = sorted(v for v in values if v is not None and math.isfinite(v))
    if not data:
        return {'n': 0}
    p95 = data[min(len(data) - 1, int(round(0.95 * (len(data) - 1))))]
    return {
        'n': len(data),
        'media': statistics.fmean(data),
        'desv': statistics.pstdev(data),
        'mediana': statistics.median(data),
        'p95': p95,
        'rmse': math.sqrt(statistics.fmean(v * v for v in data)),
        'max': data[-1],
    }
