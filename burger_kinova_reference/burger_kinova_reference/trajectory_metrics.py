# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Métricas de estabilidad de una trayectoria articular bajo degradación de red.

Responde a la pregunta PI-1 de ``docs/research/EXPERIMENTO_QOS_TELEMETRIA.md``: cómo
cambia el seguimiento de trayectorias del Kinova cuando el enlace pierde paquetes o gana
latencia y jitter. Tres familias de métricas, todas **puras** (sin ``rclpy``), para que
se prueben con ``colcon test`` sin robot:

* **Seguimiento**: error de posición que reporta el ``joint_trajectory_controller`` en su
  estado (``error.positions``), por articulación: RMS, p99 y máximo absoluto.
* **Suavidad**: *jerk* (tercera derivada de la posición medida en ``/joint_states``). Un
  enlace malo no suele aumentar el error medio, pero sí introduce tirones.
* **Cadencia**: distribución de los intervalos entre mensajes de ``/joint_states``, igual
  que ``scripts/analizar_enlace.py`` pero reutilizable.
"""

import math
from typing import Dict, List, Sequence, Tuple


def percentile(sorted_values: Sequence[float], fraction: float) -> float:
    """
    Calcular un percentil por vecino más cercano sobre valores ya ordenados.

    :param sorted_values: muestras en orden ascendente.
    :param fraction: percentil en ``[0, 1]``.
    :returns: el percentil, o ``0.0`` si no hay muestras.
    """
    if not sorted_values:
        return 0.0
    index = min(len(sorted_values) - 1,
                max(0, int(round(fraction * (len(sorted_values) - 1)))))
    return sorted_values[index]


def interval_stats(stamps: Sequence[float]) -> Dict[str, float]:
    """
    Resumir los intervalos entre marcas de tiempo consecutivas.

    :param stamps: tiempos de llegada en segundos (se ordenan internamente).
    :returns: ``n``, ``hz``, ``p50_ms``, ``p99_ms`` y ``max_ms``.
    """
    ordered = sorted(stamps)
    intervals = sorted((b - a) * 1000.0 for a, b in zip(ordered, ordered[1:]))
    duration = ordered[-1] - ordered[0] if len(ordered) > 1 else 0.0
    return {
        'n': float(len(ordered)),
        'hz': (len(ordered) - 1) / duration if duration > 0 else 0.0,
        'p50_ms': percentile(intervals, 0.50),
        'p99_ms': percentile(intervals, 0.99),
        'max_ms': intervals[-1] if intervals else 0.0,
    }


def tracking_error_stats(
    joint_names: Sequence[str],
    errors: Sequence[Sequence[float]],
) -> Dict[str, Dict[str, float]]:
    """
    Resumir el error de seguimiento por articulación.

    :param joint_names: nombres en el orden de cada fila de ``errors``.
    :param errors: una fila por mensaje de estado del controlador, en radianes.
    :returns: ``{articulación: {'rms_rad', 'p99_rad', 'max_rad'}}``.
    """
    stats: Dict[str, Dict[str, float]] = {}
    for column, name in enumerate(joint_names):
        values = [abs(row[column]) for row in errors if column < len(row)]
        values = [v for v in values if math.isfinite(v)]
        ordered = sorted(values)
        stats[name] = {
            'rms_rad': math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0,
            'p99_rad': percentile(ordered, 0.99),
            'max_rad': ordered[-1] if ordered else 0.0,
        }
    return stats


def jerk_stats(
    stamps: Sequence[float],
    positions: Sequence[Sequence[float]],
    joint_names: Sequence[str],
    min_dt_s: float = 1e-4,
) -> Dict[str, Dict[str, float]]:
    """
    Estimar el *jerk* de cada articulación por diferencias finitas de tercer orden.

    Las muestras con intervalo menor que ``min_dt_s`` se descartan: DDS a veces entrega
    dos mensajes casi simultáneos tras un hueco y dividir por un dt minúsculo inventaría
    tirones enormes. El resultado se expresa en rad/s³.

    :param stamps: tiempo de cada muestra, en segundos, ordenado.
    :param positions: una fila de posiciones por muestra, en el orden de ``joint_names``.
    :param joint_names: nombres de las articulaciones.
    :param min_dt_s: intervalo mínimo aceptado entre muestras consecutivas.
    :returns: ``{articulación: {'rms', 'max'}}``.
    """
    samples: List[Tuple[float, Sequence[float]]] = []
    for t, row in zip(stamps, positions):
        if not samples or t - samples[-1][0] >= min_dt_s:
            samples.append((t, row))

    def derivative(series: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        return [((t0 + t1) / 2.0, (v1 - v0) / (t1 - t0))
                for (t0, v0), (t1, v1) in zip(series, series[1:])]

    stats: Dict[str, Dict[str, float]] = {}
    for column, name in enumerate(joint_names):
        series = [(t, row[column]) for t, row in samples if column < len(row)]
        jerk = derivative(derivative(derivative(series)))
        values = [abs(v) for _, v in jerk if math.isfinite(v)]
        stats[name] = {
            'rms': math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0,
            'max': max(values) if values else 0.0,
        }
    return stats


def summarize_run(
    joint_stamps: Sequence[float],
    joint_positions: Sequence[Sequence[float]],
    joint_names: Sequence[str],
    error_rows: Sequence[Sequence[float]],
    error_joint_names: Sequence[str],
) -> Dict[str, float]:
    """
    Reducir una corrida a una fila de números comparables entre escenarios.

    :returns: cadencia de ``/joint_states``, peor RMS y peor máximo del error de
        seguimiento entre articulaciones, y peor RMS y peor máximo del *jerk*.
    """
    cadence = interval_stats(joint_stamps)
    tracking = tracking_error_stats(error_joint_names, error_rows)
    jerk = jerk_stats(joint_stamps, joint_positions, joint_names)
    return {
        'joint_states_hz': cadence['hz'],
        'intervalo_p99_ms': cadence['p99_ms'],
        'intervalo_max_ms': cadence['max_ms'],
        'error_rms_max_rad': max((s['rms_rad'] for s in tracking.values()), default=0.0),
        'error_abs_max_rad': max((s['max_rad'] for s in tracking.values()), default=0.0),
        'jerk_rms_max': max((s['rms'] for s in jerk.values()), default=0.0),
        'jerk_abs_max': max((s['max'] for s in jerk.values()), default=0.0),
    }
