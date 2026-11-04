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
Pruebas del relé de articulaciones hacia la escena del laboratorio.

Se comprueba contra los URDF de la escena que los nombres del driver, ya prefijados, son
exactamente las articulaciones móviles del brazo y la pinza.
"""

import os
import sys
import xml.etree.ElementTree as ET

import pytest

PAQUETE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PAQUETE, 'scripts'))

from scene_joint_relay import (  # noqa: E402
    ARTICULACION_PINZA,
    completar_articulacion,
    prefijar_nombres,
)
from sensor_msgs.msg import JointState  # noqa: E402

#: Lo que publica el driver de 6 GDL con la pinza 2F-85 en ``/joint_states``.
NOMBRES_DRIVER = [f'joint_{i}' for i in range(1, 7)] + ['robotiq_85_left_knuckle_joint']


def test_prefijo_en_orden():
    assert prefijar_nombres(['joint_1', 'joint_2'], 'gen3_') == ['gen3_joint_1', 'gen3_joint_2']


def test_no_duplica_un_prefijo_existente():
    assert prefijar_nombres(['gen3_joint_1', 'joint_2'], 'gen3_') == [
        'gen3_joint_1', 'gen3_joint_2']


def test_prefijo_vacio_no_cambia_nada():
    assert prefijar_nombres(NOMBRES_DRIVER, '') == NOMBRES_DRIVER


def _articulaciones_moviles(urdf):
    raiz = ET.parse(urdf).getroot()
    return {
        j.get('name') for j in raiz.iter('joint')
        if j.get('type') != 'fixed' and j.find('mimic') is None
    }


@pytest.mark.parametrize('urdf', ['delivery_scene_fixed.urdf', 'burger_delivery_gen3.urdf'])
def test_nombres_del_driver_coinciden_con_la_escena(urdf):
    moviles = _articulaciones_moviles(os.path.join(PAQUETE, 'urdf', urdf))
    assert set(prefijar_nombres(NOMBRES_DRIVER, 'gen3_')) == moviles


def test_completa_la_pinza_si_el_driver_no_la_publica():
    """El driver fake no trae la pinza: se añade en 0 y las listas siguen alineadas."""
    msg = JointState(name=['gen3_joint_1'], position=[0.5], velocity=[0.1], effort=[0.2])
    completar_articulacion(msg, 'gen3_' + ARTICULACION_PINZA)
    assert msg.name == ['gen3_joint_1', 'gen3_' + ARTICULACION_PINZA]
    assert list(msg.position) == [0.5, 0.0]
    assert list(msg.velocity) == [pytest.approx(0.1), 0.0]
    assert list(msg.effort) == [pytest.approx(0.2), 0.0]


def test_no_toca_la_pinza_si_el_driver_la_publica():
    """Con la pinza presente, su posición medida no se pisa con el 0."""
    nombre = 'gen3_' + ARTICULACION_PINZA
    msg = JointState(name=[nombre], position=[0.4])
    completar_articulacion(msg, nombre)
    assert msg.name == [nombre]
    assert list(msg.position) == [0.4]


def test_sin_velocidad_ni_esfuerzo_no_los_crea():
    msg = JointState(name=['gen3_joint_1'], position=[0.5])
    completar_articulacion(msg, 'gen3_' + ARTICULACION_PINZA)
    assert len(msg.velocity) == 0
    assert len(msg.effort) == 0
