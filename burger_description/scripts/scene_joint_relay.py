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
Relé de ``/joint_states`` del driver hacia la escena del laboratorio.

El driver publica las articulaciones con los nombres de ``kortex_description``
(``joint_1`` … ``joint_6``, ``robotiq_85_left_knuckle_joint``). La escena de
``burger_description`` modela el mismo brazo con todos sus frames bajo el prefijo
``gen3_``, así que su ``robot_state_publisher`` ignora esos nombres. Este nodo copia cada
mensaje a otro tópico con el prefijo añadido y deja intacto el original: el monitor, el
cliente de trayectoria y MoveIt siguen leyendo ``/joint_states`` del driver.

El driver fake se lanza sin la pinza, así que no publica ``ARTICULACION_PINZA`` y la escena se
queda sin TF para los eslabones de la pinza (``Status: Error`` en RViz). Con ``fill_gripper``,
el relé la añade en 0 (pinza abierta) si falta. Sólo debe activarse en modo fake: con el robot
real, ese 0 ocultaría una pinza que no está publicando su estado.

Lo lanza ``display.launch.py robot_real:=true``.

Sólo lee y republica estado; no envía comandos al robot.
"""

from typing import Iterable, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

#: Articulación de la pinza tal como la publica el driver; las demás son ``mimic`` de ella.
ARTICULACION_PINZA = 'robotiq_85_left_knuckle_joint'


def prefijar_nombres(nombres: Iterable[str], prefijo: str) -> List[str]:
    """
    Añadir el prefijo a cada nombre que todavía no lo tenga.

    :param nombres: nombres de articulación tal como los publica el driver.
    :param prefijo: prefijo de la escena, por ejemplo ``gen3_``.
    :returns: la lista de nombres prefijados, en el mismo orden.
    """
    return [n if n.startswith(prefijo) else prefijo + n for n in nombres]


def completar_articulacion(msg: JointState, nombre: str) -> None:
    """
    Añadir la articulación en 0 si el mensaje no la trae.

    :param msg: mensaje que se modifica; ``velocity`` y ``effort`` conservan su longitud.
    :param nombre: nombre de la articulación, ya prefijado.
    """
    if nombre in msg.name:
        return
    msg.name.append(nombre)
    msg.position.append(0.0)
    if len(msg.velocity) > 0:
        msg.velocity.append(0.0)
    if len(msg.effort) > 0:
        msg.effort.append(0.0)


class SceneJointRelay(Node):
    """Republicar ``/joint_states`` con el prefijo de la escena."""

    def __init__(self):
        super().__init__('scene_joint_relay')
        self.declare_parameter('prefix', 'gen3_')
        self.declare_parameter('input_topic', '/joint_states')
        self.declare_parameter('output_topic', '/scene/joint_states')
        self.declare_parameter('fill_gripper', False)
        self._prefijo = self.get_parameter('prefix').value
        self._completar_pinza = self.get_parameter('fill_gripper').value
        entrada = self.get_parameter('input_topic').value
        salida = self.get_parameter('output_topic').value
        self._pub = self.create_publisher(JointState, salida, 10)
        self.create_subscription(JointState, entrada, self._reenviar, 10)
        self.get_logger().info(
            f'Reenviando {entrada} -> {salida} con el prefijo "{self._prefijo}".')
        if self._completar_pinza:
            self.get_logger().warn(
                f'Modo fake: {ARTICULACION_PINZA} se publica en 0 si el driver no la envía.')

    def _reenviar(self, msg: JointState) -> None:
        msg.name = prefijar_nombres(msg.name, self._prefijo)
        if self._completar_pinza:
            completar_articulacion(msg, self._prefijo + ARTICULACION_PINZA)
        self._pub.publish(msg)


def main(args=None):
    """Arrancar el relé hasta recibir Ctrl+C."""
    rclpy.init(args=args)
    nodo = SceneJointRelay()
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
