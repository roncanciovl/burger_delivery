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

"""Pruebas de los límites de meta y del bloqueo de movimiento (RF-06, §11, PA-07)."""

from burger_kinova_connection.safety import (
    format_goal_report,
    validate_goal,
    validate_robot_ip,
    validate_safety_config,
)

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
CURRENT = {name: 0.0 for name in JOINTS}
MIN = [-3.14] * 7
MAX = [3.14] * 7


def _goal(**overrides):
    """Validar una meta usando valores seguros por defecto y las diferencias indicadas."""
    kwargs = {
        'expected_joints': JOINTS,
        'target_positions': [0.05] * 7,
        'current_positions': CURRENT,
        'joint_min': MIN,
        'joint_max': MAX,
        'max_joint_delta_rad': 0.10,
        'enable_motion': True,
        'use_fake_hardware': True,
        'telemetry_fresh': True,
        'duration_s': 5.0,
        'min_duration_s': 5.0,
    }
    kwargs.update(overrides)
    return validate_goal(**kwargs)


def test_meta_valida_se_autoriza():
    """Una meta dentro de límites, con telemetría fresca y habilitada, se autoriza."""
    result = _goal()
    assert result.allowed, result.blocks
    assert abs(result.deltas['joint_1'] - 0.05) < 1e-9


def test_bloqueo_por_movimiento_deshabilitado():
    """PA-07: con enable_motion=false la meta se bloquea localmente."""
    result = _goal(enable_motion=False)
    assert not result.allowed
    assert any('enable_motion' in block for block in result.blocks)


def test_bloqueo_menciona_hardware_real():
    """El bloqueo sobre hardware real exige autorización del laboratorio."""
    result = _goal(enable_motion=False, use_fake_hardware=False)
    assert any('HARDWARE REAL' in block for block in result.blocks)


def test_bloqueo_por_pose_incompleta():
    """PA-07: una pose con menos de siete valores se bloquea."""
    result = _goal(target_positions=[0.0, 0.0, 0.0])
    assert not result.allowed
    assert any('incompleta' in block for block in result.blocks)


def test_bloqueo_por_limite_excedido():
    """PA-07: una meta fuera de los límites aprobados se bloquea."""
    target = [0.0] * 7
    target[2] = 9.0
    result = _goal(target_positions=target, max_joint_delta_rad=100.0)
    assert not result.allowed
    assert any('excede el límite aprobado' in block for block in result.blocks)


def test_bloqueo_por_delta_excesivo():
    """Un salto mayor que max_joint_delta_rad se bloquea aunque esté dentro de límites."""
    target = [0.0] * 7
    target[0] = 1.5
    result = _goal(target_positions=target)
    assert not result.allowed
    assert any('max_joint_delta_rad' in block for block in result.blocks)


def test_bloqueo_por_telemetria_vencida():
    """Sin telemetría fresca no se construye ninguna meta."""
    result = _goal(telemetry_fresh=False)
    assert not result.allowed
    assert any('vencido' in block for block in result.blocks)


def test_bloqueo_por_posicion_actual_desconocida():
    """Si falta la posición actual de una articulación, la meta se bloquea."""
    incomplete = {name: 0.0 for name in JOINTS[:6]}
    result = _goal(current_positions=incomplete)
    assert not result.allowed
    assert any('joint_7' in block for block in result.blocks)


def test_bloqueo_por_duracion_insuficiente():
    """Una trayectoria más corta que la mínima implica mayor velocidad y se bloquea."""
    result = _goal(duration_s=1.0, min_duration_s=5.0)
    assert not result.allowed
    assert any('duración' in block for block in result.blocks)


def test_acumula_todos_los_motivos():
    """El informe reúne todos los bloqueos, no sólo el primero."""
    result = _goal(enable_motion=False, telemetry_fresh=False, target_positions=[0.0])
    assert len(result.blocks) >= 3


def test_config_valida():
    """Una configuración coherente no reporta errores."""
    assert validate_safety_config(JOINTS, [0.0] * 7, MIN, MAX, 0.10, 5.0) == []


def test_config_con_longitud_incorrecta():
    """Un arreglo de límites con longitud distinta a siete se reporta."""
    errors = validate_safety_config(JOINTS, [0.0] * 7, MIN[:3], MAX, 0.10, 5.0)
    assert any('joint_min_rad' in error for error in errors)


def test_config_con_limites_invertidos():
    """Un límite inferior mayor que el superior se reporta como inválido."""
    bad_min = list(MIN)
    bad_min[1] = 5.0
    errors = validate_safety_config(JOINTS, [0.0] * 7, bad_min, MAX, 0.10, 5.0)
    assert any('joint_2' in error for error in errors)


def test_config_con_pose_fuera_de_limites():
    """Una pose aprobada fuera de los límites configurados se rechaza al arrancar."""
    target = [0.0] * 7
    target[4] = 10.0
    errors = validate_safety_config(JOINTS, target, MIN, MAX, 0.10, 5.0)
    assert any('joint_5' in error for error in errors)


def test_config_con_delta_no_positivo():
    """max_joint_delta_rad debe ser estrictamente positivo."""
    errors = validate_safety_config(JOINTS, [0.0] * 7, MIN, MAX, 0.0, 5.0)
    assert any('max_joint_delta_rad' in error for error in errors)


def test_ip_no_se_valida_si_no_se_inicia_el_driver():
    """Como cliente DDS la IP del robot es irrelevante."""
    assert validate_robot_ip('0.0.0.0', start_driver=False, use_fake_hardware=False) == []


def test_ip_cero_valida_en_modo_fake():
    """0.0.0.0 es el valor seguro cuando se simula el hardware."""
    assert validate_robot_ip('0.0.0.0', start_driver=True, use_fake_hardware=True) == []


def test_ip_cero_invalida_con_hardware_real():
    """Con hardware real 0.0.0.0 debe rechazarse y exigirse una IP verificada."""
    errors = validate_robot_ip('0.0.0.0', start_driver=True, use_fake_hardware=False)
    assert errors and '0.0.0.0' in errors[0]


def test_ip_malformada():
    """Una cadena que no es IPv4 se reporta con claridad."""
    errors = validate_robot_ip('192.168.1', start_driver=True, use_fake_hardware=False)
    assert errors and 'IPv4' in errors[0]


def test_ip_real_valida():
    """Una IP de laboratorio verificada se acepta."""
    assert validate_robot_ip('192.168.1.10', start_driver=True, use_fake_hardware=False) == []


def test_informe_de_meta_bloqueada():
    """El informe al operador enumera cada motivo de bloqueo."""
    result = _goal(enable_motion=False)
    lines = format_goal_report(JOINTS, [0.05] * 7, CURRENT, result, 5.0)
    assert any('META BLOQUEADA' in line for line in lines)
    assert any('joint_7' in line for line in lines)
