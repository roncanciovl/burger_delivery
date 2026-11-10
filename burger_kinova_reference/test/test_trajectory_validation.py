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

"""Pruebas de la validación de trayectorias completas (planes de MoveIt 2)."""

from burger_kinova_reference.trajectory_validation import (
    format_trajectory_report,
    reorder_points,
    validate_trajectory,
)
import pytest

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
LOW = [-3.14, -2.24, -2.57, -3.14, -2.09, -3.14]
HIGH = [3.14, 2.24, 2.57, 3.14, 2.09, 3.14]
CURRENT = dict.fromkeys(JOINTS, 0.0)


def _ramp(final, duration=5.0, steps=10, start=None):
    """Trayectoria lineal desde ``start`` (ceros) hasta ``final``."""
    start = start or [0.0] * 6
    points = []
    for k in range(steps + 1):
        a = k / steps
        points.append((a * duration, [s + a * (f - s) for s, f in zip(start, final)], []))
    return points


def _validate(points, current=None, **overrides):
    kwargs = {
        'expected_joints': JOINTS, 'points': points,
        'current_positions': CURRENT if current is None else current,
        'joint_min': LOW, 'joint_max': HIGH, 'start_tolerance_rad': 0.02,
        'max_velocity_rad_s': 0.35, 'max_joint_travel_rad': 2.0, 'max_duration_s': 60.0,
        'enable_motion': True, 'use_fake_hardware': True, 'telemetry_fresh': True,
        'continuous_joints': ['joint_1', 'joint_4', 'joint_6']}
    kwargs.update(overrides)
    return validate_trajectory(**kwargs)


def test_plan_lento_y_dentro_de_limites_se_permite():
    """Un plan de 1 rad en 5 s (0.2 rad/s) cumple todas las reglas."""
    validation, summary = _validate(_ramp([1.0, 0.3, 0, 0, 0, 0]))
    assert validation.allowed, validation.blocks
    assert summary.duration_s == pytest.approx(5.0)
    assert summary.travel['joint_1'] == pytest.approx(1.0)
    assert summary.peak_velocity['joint_1'] == pytest.approx(0.2)


def test_supera_max_joint_delta_de_la_meta_unica_pero_se_permite():
    """La regla de 0.35 rad por meta no aplica a un plan completo: 1.2 rad son válidos."""
    validation, _ = _validate(_ramp([1.2, 0, 0, 0, 0, 0], duration=8.0))
    assert validation.allowed


def test_plan_desde_una_pose_vieja_se_bloquea():
    """Si el brazo ya no está donde empezó el plan, no se ejecuta."""
    actual = dict(CURRENT, joint_2=0.10)
    validation, _ = _validate(_ramp([0.5, 0, 0, 0, 0, 0]), current=actual)
    assert not validation.allowed
    assert any('joint_2' in b and 'vuelve a planificar' in b for b in validation.blocks)


def test_velocidad_excesiva_se_bloquea():
    """1 rad en 1 s supera 0.35 rad/s."""
    validation, _ = _validate(_ramp([1.0, 0, 0, 0, 0, 0], duration=1.0))
    assert any('velocidad entre puntos' in b for b in validation.blocks)


def test_velocidad_declarada_excesiva_se_bloquea():
    """Aunque las posiciones sean lentas, una velocidad declarada alta se rechaza."""
    points = [(t, q, [0.9] + [0.0] * 5) for t, q, _ in _ramp([0.5, 0, 0, 0, 0, 0])]
    validation, _ = _validate(points)
    assert any('declara' in b for b in validation.blocks)


def test_limite_articular_en_un_punto_intermedio():
    """Basta un punto fuera de límites para bloquear."""
    points = _ramp([0, 0.2, 0, 0, 0, 0], duration=20.0)
    points[5] = (points[5][0], [0, 2.30, 0, 0, 0, 0], [])
    validation, _ = _validate(points, max_velocity_rad_s=10.0, max_joint_travel_rad=10.0)
    assert any('joint_2' in b and 'fuera del límite' in b for b in validation.blocks)


def test_salto_de_dos_pi_en_articulacion_continua():
    """De +3.1 a -3.1 en joint_6 es un cambio de representación, no un giro corto."""
    start = [0, 0, 0, 0, 0, 3.1]
    current = dict(CURRENT, joint_6=3.1)
    points = [(0.0, list(start), []), (5.0, [0, 0, 0, 0, 0, -3.1], [])]
    validation, _ = _validate(points, current=current, max_velocity_rad_s=10.0,
                              max_joint_travel_rad=10.0)
    assert any('±2π' in b for b in validation.blocks)


def test_tiempos_no_crecientes_y_duracion():
    """time_from_start debe crecer y la duración total está acotada."""
    points = _ramp([0.2, 0, 0, 0, 0, 0])
    points[3] = (points[2][0], points[3][1], [])
    validation, _ = _validate(points, max_duration_s=4.0)
    assert any('estrictamente creciente' in b for b in validation.blocks)
    assert any('dura' in b for b in validation.blocks)


def test_recorrido_acumulado():
    """Ir y volver 1.5 rad recorre 3 rad aunque termine donde empezó."""
    ida = _ramp([1.5, 0, 0, 0, 0, 0], duration=10.0)
    vuelta = [(10.0 + t, q, []) for t, q, _ in
              _ramp([0, 0, 0, 0, 0, 0], duration=10.0, start=[1.5, 0, 0, 0, 0, 0])[1:]]
    validation, summary = _validate(ida + vuelta)
    assert summary.travel['joint_1'] == pytest.approx(3.0)
    assert any('recorrido acumulado' in b for b in validation.blocks)


def test_sin_habilitacion_ni_telemetria():
    """enable_motion=false y telemetría vencida se informan juntos."""
    validation, _ = _validate(_ramp([0.2, 0, 0, 0, 0, 0]), enable_motion=False,
                              use_fake_hardware=False, telemetry_fresh=False)
    assert len(validation.blocks) == 2
    assert any('HARDWARE REAL' in b for b in validation.blocks)


def test_plan_sin_movimiento_es_aviso_no_bloqueo():
    """Si el brazo ya está en la meta, el plan no se bloquea pero se avisa."""
    validation, summary = _validate([(0.0, [0.0] * 6, []), (0.1, [0.0] * 6, [])])
    assert validation.allowed and not summary.moves
    assert validation.warnings


def test_reordenar_y_detectar_articulaciones_ajenas():
    """El orden de MoveIt puede diferir; una articulación de la pinza no se acepta."""
    names = list(reversed(JOINTS))
    points = reorder_points(names, [(0.0, [6, 5, 4, 3, 2, 1], [])], JOINTS)
    assert points[0][1] == [1, 2, 3, 4, 5, 6]
    with pytest.raises(ValueError, match='sobran: robotiq'):
        reorder_points(JOINTS + ['robotiq_85_left_knuckle_joint'],
                       [(0.0, [0.0] * 7, [])], JOINTS)


def test_informe_muestra_el_resultado():
    """El informe incluye el veredicto y cada motivo de bloqueo."""
    validation, summary = _validate(_ramp([1.0, 0, 0, 0, 0, 0], duration=1.0))
    lines = format_trajectory_report(JOINTS, CURRENT, validation, summary)
    assert any('Resultado: BLOQUEADA' in line for line in lines)
    assert any('velocidad' in line for line in lines)
