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

"""Pruebas del formato de metas del ejecutor seguro de MoveIt."""

import os

from burger_kinova_reference.escena_urdf import SceneModel
from burger_kinova_reference.metas_moveit import (
    parse_metas,
    parse_secuencia,
    pose_en_marco_de_planificacion,
)
import pytest
import yaml

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'metas_moveit.yaml')
ESCENA = """
<robot name="celda">
  <link name="mesa"/><link name="world"/><link name="tag"/>
  <joint name="a" type="fixed"><parent link="mesa"/><child link="world"/>
    <origin xyz="0.2 0.3 0.025" rpy="0 0 0"/></joint>
  <joint name="b" type="fixed"><parent link="mesa"/><child link="tag"/>
    <origin xyz="0.5 0.3 0.025" rpy="0 0 0"/></joint>
</robot>
"""


def test_config_del_paquete_es_valida():
    """El YAML instalado con el paquete se interpreta y su secuencia existe."""
    with open(CONFIG, encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    metas = parse_metas(data)
    secuencia = parse_secuencia(data, metas)
    assert [m.nombre for m in secuencia][0] == 'home'
    assert metas['cerrar_pinza'].pinza == pytest.approx(0.8)


def test_errores_de_forma_nombran_la_meta():
    """Cada error dice qué meta y qué campo fallan."""
    with pytest.raises(ValueError, match='"mala".*posiciones'):
        parse_metas({'metas': {'mala': {'tipo': 'articular', 'posiciones': [0, 1]}}})
    with pytest.raises(ValueError, match='tipo'):
        parse_metas({'metas': {'x': {'tipo': 'volar'}}})
    with pytest.raises(ValueError, match='0.30 m'):
        parse_metas({'metas': {'larga': {'tipo': 'cartesiana',
                                         'desplazamiento': [0.5, 0, 0]}}})


def test_secuencia_pedida_y_desconocida():
    """La secuencia pedida por parámetro reemplaza a la del YAML."""
    metas = parse_metas({'metas': {'a': {'tipo': 'pinza', 'posicion': 0.0}}})
    assert len(parse_secuencia({}, metas, ['a', 'a'])) == 2
    with pytest.raises(ValueError, match='desconocidas'):
        parse_secuencia({'secuencia': ['b']}, metas)
    with pytest.raises(ValueError, match='vacía'):
        parse_secuencia({}, metas)


def test_pose_en_un_marco_de_la_escena():
    """0.30 m sobre el tag de la mesa, que está 0.30 m delante de la base."""
    metas = parse_metas({'metas': {'p': {'tipo': 'pose', 'marco': 'tag',
                                         'posicion': [0, 0, 0.3],
                                         'rpy': [3.14159265, 0, 0]}}})
    xyz, q = pose_en_marco_de_planificacion(metas['p'], SceneModel(ESCENA), 'world',
                                            'base_link')
    assert xyz == pytest.approx((0.3, 0.0, 0.3))
    assert abs(q[0]) == pytest.approx(1.0, abs=1e-6)


def test_pose_en_el_marco_de_planificacion_no_necesita_escena():
    """Una pose en base_link se usa tal cual."""
    metas = parse_metas({'metas': {'p': {'tipo': 'pose', 'marco': 'base_link',
                                         'posicion': [0.4, 0.1, 0.3], 'rpy': [0, 0, 0]}}})
    xyz, _ = pose_en_marco_de_planificacion(metas['p'], None, 'world', 'base_link')
    assert xyz == pytest.approx((0.4, 0.1, 0.3))
    metas['p'].marco = 'tag'
    with pytest.raises(ValueError, match='scene_urdf'):
        pose_en_marco_de_planificacion(metas['p'], None, 'world', 'base_link')
