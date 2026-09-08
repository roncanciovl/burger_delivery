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
Pruebas de la resolución de la pinza en el launch unificado.

Cubren la incompatibilidad de ``ros2_kortex`` con ROS 2 Jazzy: en modo fake el xacro del
Robotiq 2F-85 declara ``command_interface`` sobre articulaciones ``mimic`` y el
``ros2_control_node`` aborta. Con hardware real ese bloque no aparece y la pinza debe
transferirse tal cual.
"""

import importlib.util
import os

_RUTA = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'launch',
    'kinova_connection.launch.py',
)
_SPEC = importlib.util.spec_from_file_location('kinova_connection_launch', _RUTA)
_LAUNCH = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_LAUNCH)

_resolve_gripper = _LAUNCH._resolve_gripper
_as_bool = _LAUNCH._as_bool


def test_hardware_real_transfiere_la_pinza():
    """Con el robot real el 2F-85 se transfiere al bringup sin avisos."""
    pinza, aviso = _resolve_gripper('robotiq_2f_85', use_fake_hardware=False, force=False)
    assert pinza == 'robotiq_2f_85'
    assert aviso == ''


def test_modo_fake_omite_la_pinza_conflictiva():
    """En modo fake el 2F-85 se omite y se explica por qué."""
    pinza, aviso = _resolve_gripper('robotiq_2f_85', use_fake_hardware=True, force=False)
    assert pinza == ''
    assert 'mimic' in aviso
    assert 'force_gripper_in_fake' in aviso


def test_modo_fake_omite_tambien_el_2f_140():
    """El 2F-140 comparte el mismo bloque ros2_control y el mismo fallo."""
    pinza, _ = _resolve_gripper('robotiq_2f_140', use_fake_hardware=True, force=False)
    assert pinza == ''


def test_forzar_la_pinza_en_fake():
    """El override permite reproducir el fallo a propósito para diagnosticarlo."""
    pinza, aviso = _resolve_gripper('robotiq_2f_85', use_fake_hardware=True, force=True)
    assert pinza == 'robotiq_2f_85'
    assert aviso == ''


def test_valores_que_significan_sin_pinza():
    """Varias formas de pedir "brazo solo" resuelven a la cadena vacía."""
    for valor in ('', '  ', 'none', 'ninguna', 'None', 'sin_pinza', 'false'):
        pinza, aviso = _resolve_gripper(valor, use_fake_hardware=False, force=False)
        assert pinza == '', valor
        assert aviso == '', valor


def test_pinza_desconocida_se_respeta():
    """Una pinza no catalogada se transfiere tal cual, también en modo fake."""
    pinza, aviso = _resolve_gripper('gen3_lite_2f', use_fake_hardware=True, force=False)
    assert pinza == 'gen3_lite_2f'
    assert aviso == ''


def test_conversion_de_booleanos_del_launch():
    """Los valores booleanos aceptan las formas usadas en la CLI y en el YAML."""
    for valor in ('true', 'True', '1', 'yes', 'si', 'sí', 'on', True):
        assert _as_bool(valor) is True, valor
    for valor in ('false', 'False', '0', 'no', '', None, False):
        assert _as_bool(valor) is False, valor
