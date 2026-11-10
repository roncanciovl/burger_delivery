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

r"""
Carga en la escena de planificación de MoveIt los obstáculos de ``burger_description``.

Lee ``delivery_scene_fixed.urdf``, toma las cajas de colisión de los eslabones indicados
(por defecto, sólo ``table_link``), las expresa en el marco del brazo y las aplica a
``move_group`` con el servicio ``/apply_planning_scene``. Antes comprueba que ninguna caja
invada la base del robot; si alguna lo hace, **no** carga nada y termina con código 1:
MoveIt declararía en colisión el estado inicial y no planificaría.

Uso::

    ros2 run burger_kinova_reference escena_moveit
    ros2 run burger_kinova_reference escena_moveit --ros-args -p remove:=true   # quitarla
    ros2 run burger_kinova_reference escena_moveit --ros-args \
        -p obstacle_links:="['table_link', 'delivery_slot_1']"
"""

import os
import sys
import time
from typing import List, Optional

from burger_kinova_reference.escena_urdf import base_interference, SceneModel, to_pose

from geometry_msgs.msg import Pose

from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene

import rclpy
from rclpy.node import Node

from shape_msgs.msg import SolidPrimitive

EXIT_OK = 0
EXIT_BLOCKED = 1
EXIT_INFRASTRUCTURE = 2


def default_scene_path() -> str:
    """Ruta del URDF de la escena instalado por ``burger_description`` (vacía si no está)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('burger_description'), 'urdf',
                            'delivery_scene_fixed.urdf')
    except Exception:  # noqa: BLE001 - paquete no instalado
        return ''


def collision_objects(model: SceneModel, anchor: str, links: List[str], frame: str,
                      operation: int) -> List[CollisionObject]:
    """Construir un ``CollisionObject`` por eslabón, con todas sus cajas."""
    objects = []
    for link in links:
        obj = CollisionObject()
        obj.id = f'escena_{link}'
        obj.header.frame_id = frame
        obj.operation = operation
        if operation == CollisionObject.ADD:
            for size, matrix in model.collision_boxes(anchor, link):
                (x, y, z), (qx, qy, qz, qw) = to_pose(matrix)
                primitive = SolidPrimitive(type=SolidPrimitive.BOX,
                                           dimensions=[float(v) for v in size])
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = x, y, z
                (pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w) = qx, qy, qz, qw
                obj.primitives.append(primitive)
                obj.primitive_poses.append(pose)
        objects.append(obj)
    return objects


class EscenaMoveIt(Node):
    """Aplica una vez la escena de burger_description a move_group."""

    def __init__(self):
        """Declarar parámetros."""
        super().__init__('escena_moveit')
        self.declare_parameter('scene_urdf', default_scene_path())
        self.declare_parameter('anchor_link', 'world')
        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('obstacle_links', ['table_link'])
        self.declare_parameter('remove', False)
        self.declare_parameter('service', '/apply_planning_scene')
        self.declare_parameter('service_timeout_s', 30.0)

    def run(self) -> int:
        """Comprobar la escena y aplicarla; devuelve el código de salida."""
        g = self.get_parameter
        path = str(g('scene_urdf').value)
        anchor = str(g('anchor_link').value)
        frame = str(g('planning_frame').value)
        links = list(g('obstacle_links').value)
        remove = bool(g('remove').value)
        if not path or not os.path.isfile(path):
            self.get_logger().error(f'[ESCENA] No encuentro el URDF de la escena: "{path}"')
            return EXIT_INFRASTRUCTURE
        try:
            model = SceneModel.from_file(path)
            boxes = {link: model.collision_boxes(anchor, link) for link in links}
        except (KeyError, ValueError) as error:
            self.get_logger().error(f'[ESCENA] {error}')
            return EXIT_BLOCKED

        if not remove:
            problemas = base_interference(boxes)
            for problema in problemas:
                self.get_logger().error(f'[ESCENA] {problema}')
            if problemas:
                self.get_logger().error(
                    '[ESCENA] No se carga nada. Corrige la geometría en burger_description '
                    '(la base del Gen3 debe apoyarse sobre la cara superior de la mesa).')
                return EXIT_BLOCKED
            for link, shapes in boxes.items():
                for size, matrix in shapes:
                    xyz, _ = to_pose(matrix)
                    self.get_logger().info(
                        f'[ESCENA] {link}: caja {size} en {frame} '
                        f'({xyz[0]:+.3f}, {xyz[1]:+.3f}, {xyz[2]:+.3f}) m')
                if not shapes:
                    self.get_logger().warn(f'[ESCENA] {link} no tiene cajas de colisión')

        client = self.create_client(ApplyPlanningScene, str(g('service').value))
        if not client.wait_for_service(timeout_sec=float(g('service_timeout_s').value)):
            self.get_logger().error('[INFRAESTRUCTURA] move_group no ofrece '
                                    f'{g("service").value}; ¿está lanzado?')
            return EXIT_INFRASTRUCTURE
        scene = PlanningScene(is_diff=True)
        operation = CollisionObject.REMOVE if remove else CollisionObject.ADD
        scene.world.collision_objects = collision_objects(model, anchor, links, frame,
                                                          operation)
        future = client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or not future.result().success:
            self.get_logger().error('[ESCENA] move_group rechazó la escena')
            return EXIT_INFRASTRUCTURE
        accion = 'retirados' if remove else 'cargados'
        self.get_logger().info(f'[ESCENA] {len(links)} obstáculo(s) {accion}: {links}')
        return EXIT_OK


def main(args: Optional[List[str]] = None) -> None:
    """Punto de entrada del ejecutable ``escena_moveit``."""
    rclpy.init(args=args)
    node = EscenaMoveIt()
    code = EXIT_INFRASTRUCTURE
    try:
        code = node.run()
        time.sleep(0.2)
    except KeyboardInterrupt:
        code = EXIT_BLOCKED
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
