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
La escena de ``burger_description`` como fuente de la geometría de la celda para MoveIt.

El URDF ``delivery_scene_fixed.urdf`` ya describe la mesa, los tags y las zonas de entrega
respecto al brazo: el eslabón ``world`` es donde se monta la base del Gen3, es decir, el
``base_link`` del driver. Este módulo lee ese URDF **sin ROS** y produce:

* las **cajas de colisión** de los eslabones elegidos (por defecto, sólo la mesa),
  expresadas en el marco del brazo, para cargarlas en la escena de planificación;
* la **pose de cualquier marco** de la escena (``tag_mesa``, ``delivery_slot_1``...) en ese
  mismo marco, para usarlos como metas de MoveIt;
* una comprobación de **interferencia con la base**: si una caja ocupa el volumen donde se
  apoya la base del robot, MoveIt declarará en colisión el estado inicial y no planificará
  nada. Mejor detectarlo al cargar la escena que con un error críptico al planificar.

Sólo se recorren articulaciones ``fixed``: la geometría de la celda no se mueve. Las
transformaciones son matrices 4×4 en listas de Python, sin dependencias externas.
"""

import math
from typing import Dict, List, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET

Matrix = List[List[float]]
Pose = Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]  # xyz, xyzw


def _rpy_matrix(xyz: Sequence[float], rpy: Sequence[float]) -> Matrix:
    """Matriz homogénea de una traslación y una rotación roll-pitch-yaw (convención URDF)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matmul(a: Matrix, b: Matrix) -> Matrix:
    """Producto de dos matrices 4×4."""
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def invert(m: Matrix) -> Matrix:
    """Inversa de una transformación rígida 4×4."""
    rt = [[m[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(rt[i][k] * m[k][3] for k in range(3)) for i in range(3)]
    return [rt[0] + [t[0]], rt[1] + [t[1]], rt[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def to_pose(m: Matrix) -> Pose:
    """Convertir una matriz 4×4 en (posición, cuaternión xyzw)."""
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s,
             0.25 * s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
        q = (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s,
             (m[2][1] - m[1][2]) / s)
    elif m[1][1] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        q = ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s,
             (m[0][2] - m[2][0]) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
        q = ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s,
             (m[1][0] - m[0][1]) / s)
    return (m[0][3], m[1][3], m[2][3]), q


def pose_matrix(xyz: Sequence[float], rpy: Sequence[float]) -> Matrix:
    """Matriz de una pose dada como posición y roll-pitch-yaw (uso en metas)."""
    return _rpy_matrix(xyz, rpy)


def _floats(text: Optional[str], default: Sequence[float]) -> List[float]:
    return [float(v) for v in text.split()] if text else list(default)


def _origin(element) -> Matrix:
    origin = element.find('origin') if element is not None else None
    if origin is None:
        return _rpy_matrix((0, 0, 0), (0, 0, 0))
    return _rpy_matrix(_floats(origin.get('xyz'), (0, 0, 0)),
                       _floats(origin.get('rpy'), (0, 0, 0)))


class SceneModel:
    """Árbol de eslabones fijos de un URDF, con sus cajas de colisión."""

    def __init__(self, urdf_xml: str):
        """Interpretar el URDF (texto XML)."""
        root = ET.fromstring(urdf_xml)
        self.links = {link.get('name'): link for link in root.findall('link')}
        # hijo -> (padre, transformación padre->hijo, tipo de articulación)
        self.parent: Dict[str, Tuple[str, Matrix, str]] = {}
        for joint in root.findall('joint'):
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            self.parent[child] = (parent, _origin(joint), joint.get('type', 'fixed'))

    @classmethod
    def from_file(cls, path: str) -> 'SceneModel':
        """Leer el URDF desde un archivo."""
        with open(path, encoding='utf-8') as handle:
            return cls(handle.read())

    def _chain_to_root(self, link: str) -> Tuple[List[str], Matrix]:
        """Transformación raíz->``link`` y la lista de eslabones recorridos."""
        if link not in self.links:
            raise KeyError(f'el eslabón "{link}" no existe en la escena')
        transform = _rpy_matrix((0, 0, 0), (0, 0, 0))
        path = [link]
        current = link
        while current in self.parent:
            parent, origin, kind = self.parent[current]
            if kind != 'fixed':
                raise ValueError(f'la cadena hasta "{link}" pasa por la articulación móvil '
                                 f'"{kind}" entre {parent} y {current}')
            transform = matmul(origin, transform)
            current = parent
            path.append(current)
        return path, transform

    def transform(self, anchor: str, link: str) -> Matrix:
        """Transformación ``anchor -> link`` (la pose de ``link`` expresada en ``anchor``)."""
        _, root_anchor = self._chain_to_root(anchor)
        _, root_link = self._chain_to_root(link)
        return matmul(invert(root_anchor), root_link)

    def collision_boxes(self, anchor: str, link: str) -> List[Tuple[List[float], Matrix]]:
        """
        Cajas de colisión de ``link`` en el marco ``anchor``: lista de (tamaño xyz, pose).

        Las colisiones con malla se ignoran: en la escena son marcadores visuales.
        """
        boxes = []
        base = self.transform(anchor, link)
        for collision in self.links[link].findall('collision'):
            box = collision.find('geometry/box')
            if box is None:
                continue
            boxes.append((_floats(box.get('size'), (0, 0, 0)),
                          matmul(base, _origin(collision))))
        return boxes


def box_overlaps_region(size: Sequence[float], pose: Matrix,
                        region_min: Sequence[float], region_max: Sequence[float]) -> bool:
    """
    Indicar si una caja (posiblemente rotada) se superpone a una región alineada a los ejes.

    Se compara la caja envolvente alineada a ejes de la caja rotada: es conservador
    (puede avisar de más en cajas muy rotadas), nunca de menos.
    """
    half = [s / 2.0 for s in size]
    extent = [sum(abs(pose[i][k]) * half[k] for k in range(3)) for i in range(3)]
    for i in range(3):
        if pose[i][3] + extent[i] <= region_min[i] or pose[i][3] - extent[i] >= region_max[i]:
            return False
    return True


def base_interference(boxes: Dict[str, List[Tuple[List[float], Matrix]]],
                      radius_m: float = 0.06, height_m: float = 0.10) -> List[str]:
    """
    Detectar cajas que ocupan el volumen de la base del robot.

    La base se aproxima por un prisma de ``2·radius_m`` de lado sobre el plano z = 0 del
    marco del brazo (de 1 mm a ``height_m``). Una mesa modelada con su cara superior por
    encima de z = 0 cae aquí: la base quedaría "enterrada" y MoveIt no planificaría.

    :returns: un mensaje por caja en conflicto, con la penetración en z.
    """
    problems = []
    region_min = (-radius_m, -radius_m, 0.001)
    region_max = (radius_m, radius_m, height_m)
    for name, shapes in boxes.items():
        for size, pose in shapes:
            if box_overlaps_region(size, pose, region_min, region_max):
                top = pose[2][3] + sum(abs(pose[2][k]) * size[k] / 2.0 for k in range(3))
                problems.append(
                    f'"{name}" ocupa el volumen de la base del robot: su cara superior está '
                    f'en z = {top:+.3f} m respecto a la base (debería ser ≤ 0). MoveIt '
                    'declarará en colisión el estado inicial.')
    return problems
