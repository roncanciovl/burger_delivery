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

"""Pruebas de los marcadores con las cajas de colisión de la escena del laboratorio."""

import math
import os
import sys

import pytest

PAQUETE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PAQUETE, 'scripts'))

from scene_collision_markers import (  # noqa: E402
    cajas_de_colision,
    marcadores,
    rpy_a_cuaternion,
)

ESCENA = os.path.join(PAQUETE, 'urdf', 'delivery_scene_fixed.urdf')

URDF = """
<robot name="celda">
  <link name="mesa">
    <collision><origin xyz="0.1 0.2 0.3" rpy="0 0 1.5707963267948966"/>
      <geometry><box size="1 2 3"/></geometry></collision>
    <collision><geometry><mesh filename="package://x/marcador.stl"/></geometry></collision>
  </link>
  <link name="sin_origen">
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
  <link name="vacio"/>
</robot>
"""


def test_la_mesa_de_la_escena_tiene_su_caja_de_colision():
    """La caja mide lo que la mesa real y baja de su cara superior al piso, cubriendo las patas."""
    with open(ESCENA, encoding='utf-8') as archivo:
        cajas = cajas_de_colision(archivo.read(), ['table_link'])
    assert len(cajas) == 1
    assert cajas[0].tamano == pytest.approx((2.0, 0.9, 0.92))
    assert cajas[0].posicion[:2] == pytest.approx((0.0, 0.0))
    assert cajas[0].posicion[2] + cajas[0].tamano[2] / 2.0 == pytest.approx(0.025)


def test_lee_tamano_y_pose_e_ignora_mallas():
    caja, = cajas_de_colision(URDF, ['mesa'])
    assert caja.tamano == (1.0, 2.0, 3.0)
    assert caja.posicion == (0.1, 0.2, 0.3)
    assert caja.cuaternion == pytest.approx((0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))


def test_origen_omitido_es_la_identidad():
    caja, = cajas_de_colision(URDF, ['sin_origen'])
    assert caja.posicion == (0.0, 0.0, 0.0)
    assert caja.cuaternion == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_eslabon_sin_colision_no_produce_cajas():
    assert cajas_de_colision(URDF, ['vacio']) == []


def test_eslabon_inexistente_falla():
    with pytest.raises(KeyError, match='no_existe'):
        cajas_de_colision(URDF, ['no_existe'])


def test_cuaternion_de_roll_y_pitch():
    assert rpy_a_cuaternion(math.pi, 0.0, 0.0) == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-9)
    assert rpy_a_cuaternion(0.0, math.pi, 0.0) == pytest.approx((0.0, 1.0, 0.0, 0.0), abs=1e-9)


def test_marcador_anclado_al_eslabon_y_translucido():
    marcador, = marcadores(cajas_de_colision(URDF, ['mesa'])).markers
    assert marcador.header.frame_id == 'mesa'
    assert (marcador.scale.x, marcador.scale.y, marcador.scale.z) == (1.0, 2.0, 3.0)
    assert (marcador.pose.position.x, marcador.pose.position.y,
            marcador.pose.position.z) == (0.1, 0.2, 0.3)
    assert 0.0 < marcador.color.a < 1.0


def test_marcadores_no_repiten_identificador():
    cajas = cajas_de_colision(URDF, ['mesa', 'sin_origen'])
    ids = [marcador.id for marcador in marcadores(cajas).markers]
    assert len(ids) == len(set(ids)) == 2
