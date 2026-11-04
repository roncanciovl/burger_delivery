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

"""Pruebas de la lectura de la escena de burger_description para MoveIt."""

import math
import os

from burger_kinova_reference.escena_urdf import (
    base_interference,
    invert,
    matmul,
    pose_matrix,
    SceneModel,
    to_pose,
)
import pytest

ESCENA = """
<robot name="celda">
  <link name="map"/>
  <link name="mesa">
    <collision><origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="1.0 0.6 0.05"/></geometry></collision>
    <collision><geometry><mesh filename="package://x/marcador.stl"/></geometry></collision>
  </link>
  <link name="world"/>
  <link name="tag"/>
  <link name="brazo"/>
  <joint name="map_mesa" type="fixed"><parent link="map"/><child link="mesa"/>
    <origin xyz="1 2 0.8" rpy="0 0 1.5707963267948966"/></joint>
  <joint name="mesa_world" type="fixed"><parent link="mesa"/><child link="world"/>
    <origin xyz="0.2 0.1 {z}" rpy="0 0 0"/></joint>
  <joint name="mesa_tag" type="fixed"><parent link="mesa"/><child link="tag"/>
    <origin xyz="0.4 0.1 0.025" rpy="0 0 0"/></joint>
  <joint name="j1" type="revolute"><parent link="world"/><child link="brazo"/></joint>
</robot>
"""


def _modelo(z=0.025):
    return SceneModel(ESCENA.format(z=z))


def test_pose_de_un_marco_en_el_marco_del_brazo():
    """El tag queda 0.2 m delante de la base, a la altura de la cara de la mesa."""
    xyz, q = to_pose(_modelo().transform('world', 'tag'))
    assert xyz == pytest.approx((0.2, 0.0, 0.0))
    assert q == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_transformacion_inversa_y_rotacion():
    """La rotación de map->mesa se refleja en la pose de map vista desde el brazo."""
    modelo = _modelo()
    ida = modelo.transform('world', 'map')
    vuelta = modelo.transform('map', 'world')
    identidad = matmul(ida, vuelta)
    for i in range(4):
        assert identidad[i] == pytest.approx([1.0 if i == j else 0.0 for j in range(4)])
    assert to_pose(invert(ida))[0] == pytest.approx((0.9, 2.2, 0.825))


def test_cajas_de_colision_ignoran_las_mallas():
    """Sólo la caja de la mesa; la malla del marcador no es un obstáculo."""
    cajas = _modelo().collision_boxes('world', 'mesa')
    assert len(cajas) == 1
    tamano, pose = cajas[0]
    assert tamano == [1.0, 0.6, 0.05]
    assert to_pose(pose)[0] == pytest.approx((-0.2, -0.1, -0.025))


def test_interferencia_con_la_base():
    """Con la base a la altura del centro de la mesa, la caja la invade 25 mm."""
    enterrada = {'mesa': _modelo(z=0.0).collision_boxes('world', 'mesa')}
    problemas = base_interference(enterrada)
    assert len(problemas) == 1 and '+0.025' in problemas[0]
    apoyada = {'mesa': _modelo(z=0.025).collision_boxes('world', 'mesa')}
    assert base_interference(apoyada) == []


def test_cadena_con_articulacion_movil_se_rechaza():
    """La geometría de la celda sólo usa articulaciones fijas."""
    with pytest.raises(ValueError, match='móvil'):
        _modelo().transform('world', 'brazo')
    with pytest.raises(KeyError):
        _modelo().transform('world', 'no_existe')


def test_pose_matrix_roundtrip():
    """Una pose con giro de 180° en x (pinza hacia abajo) da el cuaternión esperado."""
    _, q = to_pose(pose_matrix((0.3, 0, 0.15), (math.pi, 0, 0)))
    assert [abs(v) for v in q] == pytest.approx([1.0, 0.0, 0.0, 0.0], abs=1e-9)


ESCENA_REAL = os.path.join(os.path.dirname(__file__), '..', '..', 'burger_description',
                           'urdf', 'delivery_scene_fixed.urdf')


@pytest.mark.skipif(not os.path.isfile(ESCENA_REAL), reason='burger_description no está al lado')
def test_escena_real_de_burger_description():
    """La base del Gen3 se apoya sobre la mesa y tag_mesa está sobre su cara superior."""
    modelo = SceneModel.from_file(ESCENA_REAL)
    assert base_interference({'table_link': modelo.collision_boxes('world', 'table_link')}) == []
    xyz, _ = to_pose(modelo.transform('world', 'tag_mesa'))
    assert xyz == pytest.approx((0.30, 0.0, 0.0))


@pytest.mark.skipif(not os.path.isfile(ESCENA_REAL), reason='burger_description no está al lado')
def test_base_al_borde_del_lado_corto_de_la_mesa():
    """La base va al centro del lado de 0.9 m y la mesa de 2 m se extiende hacia +x."""
    modelo = SceneModel.from_file(ESCENA_REAL)
    (tamano, pose), = modelo.collision_boxes('world', 'table_link')
    (x, y, z), _ = to_pose(pose)
    assert tamano == pytest.approx([2.0, 0.9, 0.92])
    assert x - tamano[0] / 2.0 == pytest.approx(-0.06)
    assert y == pytest.approx(0.0)
    assert z + tamano[2] / 2.0 == pytest.approx(0.0)
