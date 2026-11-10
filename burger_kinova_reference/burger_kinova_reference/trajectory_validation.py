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
Validación de una trayectoria COMPLETA antes de ejecutarla (planes de MoveIt 2).

:func:`burger_kinova_reference.safety.validate_goal` decide sobre **una** meta cercana: ningún
eje puede alejarse más de ``max_joint_delta_rad`` de la pose actual. Un plan de MoveIt es
otra cosa: decenas de puntos con tiempos, que pueden recorrer más de un radián rodeando un
obstáculo. Aplicarle la regla de la meta única lo rechazaría siempre; no aplicarle ninguna
regla dejaría que el planificador decidiera solo qué movimiento llega al robot.

Este módulo traslada las reglas del laboratorio a la trayectoria entera. Como el resto del
package, es lógica pura: se prueba con ``colcon test`` sin robot ni MoveIt.

Reglas (todas se evalúan y se informan juntas):

* **Articulaciones**: la trayectoria trae exactamente las articulaciones esperadas.
* **Tiempo**: ``time_from_start`` estrictamente creciente; duración total acotada.
* **Arranque**: el primer punto coincide con la pose MEDIDA (``start_tolerance_rad``). Un
  plan calculado sobre una pose vieja movería el brazo desde donde ya no está.
* **Límites**: todos los puntos dentro de ``joint_min_rad``/``joint_max_rad``.
* **Velocidad**: la velocidad entre puntos consecutivos, y la declarada en cada punto, no
  superan ``max_velocity_rad_s``.
* **Saltos de 2π**: en una articulación continua, un paso mayor que π entre dos puntos es
  un cambio de representación del ángulo, no un movimiento real; el controlador lo
  ejecutaría como una vuelta casi completa.
* **Recorrido**: la suma de los desplazamientos de cada articulación está acotada.
* **Habilitación**: telemetría fresca y ``enable_motion``, igual que en ``validate_goal``.
"""

import math
from typing import Dict, List, Optional, Sequence, Tuple

from burger_kinova_reference.safety import GoalValidation

#: (tiempo desde el inicio [s], posiciones [rad], velocidades [rad/s] o lista vacía)
TrajectoryPoint = Tuple[float, List[float], List[float]]

#: Desplazamiento por debajo del cual se considera que el plan no mueve el brazo.
NO_MOTION_RAD = 1e-4


class TrajectorySummary:
    """Números de la trayectoria que se muestran al operador antes de ejecutarla."""

    def __init__(self, points: int, duration_s: float, travel: Dict[str, float],
                 peak_velocity: Dict[str, float], final: Dict[str, float]):
        """
        Construir el resumen.

        :param points: número de puntos de la trayectoria.
        :param duration_s: ``time_from_start`` del último punto.
        :param travel: recorrido acumulado por articulación, en radianes.
        :param peak_velocity: velocidad máxima entre puntos por articulación, en rad/s.
        :param final: posición final por articulación.
        """
        self.points = points
        self.duration_s = duration_s
        self.travel = travel
        self.peak_velocity = peak_velocity
        self.final = final

    @property
    def moves(self) -> bool:
        """Indicar si la trayectoria mueve alguna articulación de forma apreciable."""
        return any(value > NO_MOTION_RAD for value in self.travel.values())


def reorder_points(
    joint_names: Sequence[str],
    points: Sequence[TrajectoryPoint],
    expected_joints: Sequence[str],
) -> List[TrajectoryPoint]:
    """
    Expresar los puntos en el orden de ``expected_joints``.

    MoveIt ordena las articulaciones según el grupo de planificación, que no tiene por
    qué coincidir con el de la configuración.

    :raises ValueError: si faltan articulaciones esperadas o sobran otras.
    """
    names = list(joint_names)
    missing = [joint for joint in expected_joints if joint not in names]
    extra = [joint for joint in names if joint not in expected_joints]
    if missing or extra:
        raise ValueError(
            'la trayectoria no trae las articulaciones esperadas'
            + (f'; faltan: {", ".join(missing)}' if missing else '')
            + (f'; sobran: {", ".join(extra)}' if extra else ''))
    index = [names.index(joint) for joint in expected_joints]
    reordered = []
    for t, positions, velocities in points:
        reordered.append((
            float(t),
            [float(positions[i]) for i in index],
            [float(velocities[i]) for i in index] if len(velocities) == len(names) else [],
        ))
    return reordered


def summarize_trajectory(
    expected_joints: Sequence[str],
    points: Sequence[TrajectoryPoint],
) -> TrajectorySummary:
    """Calcular duración, recorrido y velocidad pico de una trayectoria ya reordenada."""
    travel = dict.fromkeys(expected_joints, 0.0)
    peak = dict.fromkeys(expected_joints, 0.0)
    for (t0, q0, _), (t1, q1, _) in zip(points, points[1:]):
        dt = t1 - t0
        for i, joint in enumerate(expected_joints):
            step = abs(q1[i] - q0[i])
            travel[joint] += step
            if dt > 0.0:
                peak[joint] = max(peak[joint], step / dt)
    final = ({joint: points[-1][1][i] for i, joint in enumerate(expected_joints)}
             if points else {})
    return TrajectorySummary(
        points=len(points),
        duration_s=points[-1][0] if points else 0.0,
        travel=travel,
        peak_velocity=peak,
        final=final,
    )


def validate_trajectory(
    expected_joints: Sequence[str],
    points: Sequence[TrajectoryPoint],
    current_positions: Dict[str, float],
    joint_min: Sequence[float],
    joint_max: Sequence[float],
    start_tolerance_rad: float,
    max_velocity_rad_s: float,
    max_joint_travel_rad: float,
    max_duration_s: float,
    enable_motion: bool,
    use_fake_hardware: bool,
    telemetry_fresh: bool,
    continuous_joints: Optional[Sequence[str]] = None,
) -> Tuple[GoalValidation, TrajectorySummary]:
    """
    Decidir si una trayectoria completa puede enviarse al controlador.

    :param expected_joints: articulaciones esperadas, en el orden de la configuración.
    :param points: puntos ya reordenados con :func:`reorder_points`.
    :param current_positions: última posición válida medida por articulación.
    :param joint_min: límite inferior aprobado por articulación.
    :param joint_max: límite superior aprobado por articulación.
    :param start_tolerance_rad: diferencia máxima entre el primer punto y la pose medida.
    :param max_velocity_rad_s: velocidad articular máxima aceptada.
    :param max_joint_travel_rad: recorrido acumulado máximo por articulación.
    :param max_duration_s: duración máxima aceptada de la trayectoria.
    :param enable_motion: habilitación explícita del operador.
    :param use_fake_hardware: ``True`` si se opera con hardware simulado.
    :param telemetry_fresh: ``True`` si ``/joint_states`` está dentro del *timeout*.
    :param continuous_joints: articulaciones sin tope, donde se vigilan los saltos de 2π.
    :returns: ``(validación, resumen)``.
    """
    blocks: List[str] = []
    warnings: List[str] = []
    n = len(expected_joints)
    continuous = set(continuous_joints or [])
    summary = summarize_trajectory(expected_joints, points)

    if len(joint_min) != n or len(joint_max) != n:
        blocks.append(
            f'los límites articulares no cubren las {n} articulaciones esperadas '
            f'(joint_min_rad={len(joint_min)}, joint_max_rad={len(joint_max)})')
    if not points:
        blocks.append('la trayectoria está vacía')
    if not telemetry_fresh:
        blocks.append('el estado articular está vencido o ausente: no se puede comprobar '
                      'que el plan arranque donde está el brazo')
    missing = [joint for joint in expected_joints if joint not in current_positions]
    if missing:
        blocks.append(f'faltan posiciones actuales de: {", ".join(missing)}')

    if points:
        if any(len(q) != n for _, q, _ in points):
            blocks.append('hay puntos con un número de posiciones distinto al de articulaciones')
        else:
            _check_times(points, max_duration_s, blocks)
            _check_start(expected_joints, points[0][1], current_positions,
                         start_tolerance_rad, blocks)
            if len(joint_min) == n and len(joint_max) == n:
                _check_limits(expected_joints, points, joint_min, joint_max, blocks)
            _check_motion(expected_joints, points, summary, continuous, max_velocity_rad_s,
                          max_joint_travel_rad, blocks)
            if not summary.moves:
                warnings.append('la trayectoria no mueve el brazo: el robot ya está en la meta')

    if not enable_motion:
        if use_fake_hardware:
            blocks.append('movimiento deshabilitado (enable_motion=false). En modo fake '
                          'habilítalo de forma explícita para completar la prueba')
        else:
            blocks.append('movimiento deshabilitado (enable_motion=false) sobre HARDWARE '
                          'REAL: requiere autorización del responsable del laboratorio, '
                          'espacio despejado y parada de emergencia accesible')

    deltas = {joint: summary.final[joint] - current_positions[joint]
              for joint in expected_joints
              if joint in summary.final and joint in current_positions}
    return (GoalValidation(allowed=not blocks, blocks=blocks, warnings=warnings,
                           deltas=deltas), summary)


def _check_times(points, max_duration_s, blocks):
    times = [t for t, _, _ in points]
    if times[0] < 0.0 or not all(math.isfinite(t) for t in times):
        blocks.append('time_from_start inválido en el primer punto o no finito')
    backwards = [i for i in range(1, len(times)) if times[i] <= times[i - 1]]
    if backwards:
        blocks.append(f'time_from_start no es estrictamente creciente (punto {backwards[0]})')
    if times[-1] > max_duration_s:
        blocks.append(f'la trayectoria dura {times[-1]:.2f} s y el máximo aceptado es '
                      f'{max_duration_s:.2f} s')


def _check_start(expected_joints, first, current, tolerance, blocks):
    for i, joint in enumerate(expected_joints):
        if joint not in current:
            continue
        gap = abs(first[i] - float(current[joint]))
        if gap > tolerance:
            blocks.append(
                f'{joint}: el plan arranca en {first[i]:+.4f} rad y el brazo está en '
                f'{float(current[joint]):+.4f} rad (diferencia {gap:.4f} > '
                f'{tolerance:.4f}); vuelve a planificar desde la pose actual')


def _check_limits(expected_joints, points, joint_min, joint_max, blocks):
    for i, joint in enumerate(expected_joints):
        low, high = float(joint_min[i]), float(joint_max[i])
        values = [q[i] for _, q, _ in points]
        if not all(math.isfinite(v) for v in values):
            blocks.append(f'{joint}: hay posiciones no finitas en la trayectoria')
            continue
        worst = min(values) if min(values) < low else max(values)
        if not (low <= min(values) and max(values) <= high):
            blocks.append(f'{joint}: la trayectoria alcanza {worst:+.4f} rad, fuera del '
                          f'límite aprobado [{low:.4f}, {high:.4f}]')


def _check_motion(expected_joints, points, summary, continuous, max_velocity,
                  max_travel, blocks):
    for i, joint in enumerate(expected_joints):
        for k in range(1, len(points)):
            step = points[k][1][i] - points[k - 1][1][i]
            if joint in continuous and abs(step) > math.pi:
                blocks.append(
                    f'{joint}: salto de {step:+.3f} rad entre los puntos {k - 1} y {k}; en '
                    'una articulación continua es un cambio de representación (±2π), y el '
                    'controlador lo ejecutaría como casi una vuelta completa')
                break
        if summary.peak_velocity[joint] > max_velocity:
            blocks.append(f'{joint}: velocidad entre puntos de '
                          f'{summary.peak_velocity[joint]:.3f} rad/s supera '
                          f'max_velocity_rad_s={max_velocity:.3f}')
        declared = max((abs(v[i]) for _, _, v in points if v), default=0.0)
        if declared > max_velocity:
            blocks.append(f'{joint}: el plan declara {declared:.3f} rad/s y el máximo es '
                          f'{max_velocity:.3f}; reduce el escalado de velocidad de MoveIt')
        if summary.travel[joint] > max_travel:
            blocks.append(f'{joint}: recorrido acumulado {summary.travel[joint]:.3f} rad '
                          f'supera max_joint_travel_rad={max_travel:.3f}')


def format_trajectory_report(
    expected_joints: Sequence[str],
    current_positions: Dict[str, float],
    validation: GoalValidation,
    summary: TrajectorySummary,
) -> List[str]:
    """Líneas del informe que se muestra antes de pedir confirmación al operador."""
    lines = [
        '──────── Validación de la trayectoria planificada ────────',
        f'  Puntos: {summary.points} · duración: {summary.duration_s:.2f} s',
        '  Articulación   actual     final    recorrido   v. pico',
    ]
    for joint in expected_joints:
        current = current_positions.get(joint, float('nan'))
        final = summary.final.get(joint, float('nan'))
        lines.append(
            f'  {joint:<12} {current:+8.4f}  {final:+8.4f}'
            f'  {summary.travel.get(joint, 0.0):8.4f}'
            f'  {summary.peak_velocity.get(joint, 0.0):7.3f}')
    for warning in validation.warnings:
        lines.append(f'  AVISO: {warning}')
    if validation.allowed:
        lines.append('  Resultado: PERMITIDA')
    else:
        lines.append('  Resultado: BLOQUEADA')
        lines.extend(f'    - {block}' for block in validation.blocks)
    return lines
