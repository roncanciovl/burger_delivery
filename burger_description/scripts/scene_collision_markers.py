#!/usr/bin/env python3
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
Muestra en RViz las cajas de colisión de la escena del laboratorio.

El display ``RobotModel`` no sirve para esto: dibuja las colisiones de **todos** los
eslabones, incluidos los que MoveIt no carga, y como el modelo le llega por tópico después
de abrir la configuración, ignora lo guardado por eslabón (``Alpha``, ``All Links Enabled``).

Este nodo lee las cajas del URDF de la escena, el mismo del que ``escena_moveit`` toma los
obstáculos, y las publica una vez como marcadores. Por defecto, sólo ``table_link``, igual
que ``escena_moveit``.

Lo lanza ``display.launch.py``. No envía comandos al robot.
"""

import math
import os
from typing import Iterable, List, NamedTuple, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

#: Rojo translúcido: se distingue del material de la mesa y deja ver el brazo al otro lado.
COLOR_COLISION = (1.0, 0.1, 0.1, 0.45)


class Caja(NamedTuple):
    """Caja de colisión en el marco de su eslabón."""

    eslabon: str
    tamano: Tuple[float, float, float]
    posicion: Tuple[float, float, float]
    cuaternion: Tuple[float, float, float, float]  # xyzw


def _floats(texto: Optional[str], defecto: Sequence[float]) -> Tuple[float, ...]:
    return tuple(float(v) for v in texto.split()) if texto else tuple(defecto)


def rpy_a_cuaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convertir roll-pitch-yaw (convención URDF) en un cuaternión xyzw."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def cajas_de_colision(urdf_xml: str, eslabones: Iterable[str]) -> List[Caja]:
    """
    Leer las cajas de colisión de los eslabones indicados.

    Las colisiones con malla se ignoran, como en ``escena_moveit``: MoveIt tampoco las carga.

    :raises KeyError: si un eslabón no existe en el URDF.
    """
    links = {link.get('name'): link for link in ET.fromstring(urdf_xml).findall('link')}
    cajas = []
    for nombre in eslabones:
        if nombre not in links:
            raise KeyError(f'el eslabón "{nombre}" no existe en la escena')
        for colision in links[nombre].findall('collision'):
            caja = colision.find('geometry/box')
            if caja is None:
                continue
            origen = colision.find('origin')
            xyz = origen.get('xyz') if origen is not None else None
            rpy = origen.get('rpy') if origen is not None else None
            cajas.append(Caja(nombre,
                              _floats(caja.get('size'), (0.0, 0.0, 0.0)),
                              _floats(xyz, (0.0, 0.0, 0.0)),
                              rpy_a_cuaternion(*_floats(rpy, (0.0, 0.0, 0.0)))))
    return cajas


def marcadores(cajas: Iterable[Caja]) -> MarkerArray:
    """Construir un marcador por caja, anclado al marco de su eslabón."""
    mensaje = MarkerArray()
    for indice, caja in enumerate(cajas):
        marcador = Marker()
        marcador.header.frame_id = caja.eslabon
        marcador.ns = caja.eslabon
        marcador.id = indice
        marcador.type = Marker.CUBE
        marcador.action = Marker.ADD
        marcador.scale.x, marcador.scale.y, marcador.scale.z = caja.tamano
        marcador.pose.position.x, marcador.pose.position.y, marcador.pose.position.z = \
            caja.posicion
        (marcador.pose.orientation.x, marcador.pose.orientation.y,
         marcador.pose.orientation.z, marcador.pose.orientation.w) = caja.cuaternion
        marcador.color.r, marcador.color.g, marcador.color.b, marcador.color.a = COLOR_COLISION
        mensaje.markers.append(marcador)
    return mensaje


class SceneCollisionMarkers(Node):
    """Publicar una vez, con durabilidad, las cajas de colisión de la escena."""

    def __init__(self):
        super().__init__('scene_collision_markers')
        self.declare_parameter('scene_urdf', os.path.join(
            get_package_share_directory('burger_description'), 'urdf',
            'delivery_scene_fixed.urdf'))
        self.declare_parameter('links', ['table_link'])
        self.declare_parameter('topic', '/scene/collision_markers')
        ruta = self.get_parameter('scene_urdf').value
        eslabones = list(self.get_parameter('links').value)
        topico = self.get_parameter('topic').value
        with open(ruta, encoding='utf-8') as archivo:
            cajas = cajas_de_colision(archivo.read(), eslabones)
        if not cajas:
            self.get_logger().warn(f'{eslabones} no tienen cajas de colisión')
        # Durabilidad: RViz recibe los marcadores aunque se abra después de este nodo.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(MarkerArray, topico, qos)
        self._pub.publish(marcadores(cajas))
        for caja in cajas:
            self.get_logger().info(f'{caja.eslabon}: caja {list(caja.tamano)} en {topico}')


def main(args=None):
    """Mantener publicados los marcadores hasta recibir Ctrl+C."""
    rclpy.init(args=args)
    nodo = SceneCollisionMarkers()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
