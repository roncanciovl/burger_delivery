# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""
Servidor de la acción ``/<car>/prepare_delivery_pose`` (TODO.md §3).

Lleva un carrito a una zona de entrega (*delivery slot*) y confirma con la localización
absoluta (cadena ``tag_mesa -> tag_carrito -> car_base_link``) que llegó dentro de la
tolerancia, antes de que el Kinova le deposite la caja.

1. Resuelve el destino: ``slot_id`` de ``config/delivery_slots.yaml`` o ``target_pose``.
2. Lo expresa en ``nav_frame`` (``map``) y lo envía a Nav2 (``/<car>/navigate_to_pose``),
   reenviando ``distance_remaining`` como feedback. Con ``use_nav2:=false`` no navega: el
   carrito se mueve por su cuenta (o a mano) y el servidor sólo verifica.
3. Espera ``settle_time_s`` y verifica la pose con TF. Si queda fuera de tolerancia,
   reintenta la navegación hasta ``max_retries`` veces.

Cancelar la acción cancela la meta de Nav2.
"""

import math
import threading
import time

from action_msgs.msg import GoalStatus
from burger_interfaces.action import PrepareDeliveryPose
from burger_navigation.delivery_logic import (
    compose,
    parse_slots,
    pose_error,
    quaternion_from_yaw,
    resolve_tolerance,
    yaw_from_quaternion,
)
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


def _pose2_of_msg(pose):
    q = pose.orientation
    return pose.position.x, pose.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)


def _pose_msg(frame, pose2, stamp):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    msg.pose.position.x, msg.pose.position.y = pose2[0], pose2[1]
    (msg.pose.orientation.x, msg.pose.orientation.y,
     msg.pose.orientation.z, msg.pose.orientation.w) = quaternion_from_yaw(pose2[2])
    return msg


class DeliveryPoseServer(Node):
    """Acción PrepareDeliveryPose sobre Nav2 con verificación por AprilTag."""

    def __init__(self):
        """Declarar parámetros, cargar las zonas de entrega y crear la acción."""
        super().__init__('delivery_pose_server')
        p = self.declare_parameter
        p('slots_file', '')
        p('car_base_frame', 'car1_base_link')
        p('nav_frame', 'map')
        p('use_nav2', True)
        p('nav2_action', 'navigate_to_pose')
        p('nav2_timeout_s', 120.0)
        p('default_tolerance_xy', 0.03)
        p('default_tolerance_yaw_deg', 8.0)
        p('settle_time_s', 1.0)
        p('max_retries', 1)

        g = self.get_parameter
        self.slots = {}
        if g('slots_file').value:
            with open(g('slots_file').value, encoding='utf-8') as f:
                self.slots = parse_slots(yaml.safe_load(f).get('delivery_slots', {}))
        self.car_frame = g('car_base_frame').value
        self.nav_frame = g('nav_frame').value
        self.use_nav2 = g('use_nav2').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        grupo = ReentrantCallbackGroup()
        self.nav = ActionClient(self, NavigateToPose, g('nav2_action').value,
                                callback_group=grupo)
        self.server = ActionServer(
            self, PrepareDeliveryPose, 'prepare_delivery_pose', self.execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=grupo)
        self.get_logger().info(
            f'prepare_delivery_pose listo: {len(self.slots)} slots {sorted(self.slots)}, '
            f'carrito {self.car_frame}, Nav2 {"sí" if self.use_nav2 else "no (sólo verifica)"}')

    # --- utilidades TF ------------------------------------------------------------------
    def _lookup2(self, parent, child, timeout=1.0):
        tf = self.tf_buffer.lookup_transform(parent, child, Time(),
                                             timeout=Duration(seconds=timeout))
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _car_pose(self, frame):
        """Pose del carrito en ``frame`` según la cadena de TF (None si no hay)."""
        try:
            return self._lookup2(frame, self.car_frame)
        except TransformException as error:
            self.get_logger().warn(f'Sin TF {frame} -> {self.car_frame}: {error}',
                                   throttle_duration_sec=5.0)
            return None

    # --- acción -------------------------------------------------------------------------
    def execute(self, goal_handle):
        """Ejecutar una meta de PrepareDeliveryPose."""
        goal = goal_handle.request
        result = PrepareDeliveryPose.Result()

        def terminar(ok, mensaje, frame=None, pose=None, err=(math.nan, math.nan)):
            result.success, result.status_message = ok, mensaje
            if pose is not None:
                result.final_pose = _pose_msg(frame, pose, self.get_clock().now().to_msg())
            result.error_xy, result.error_yaw = err
            (goal_handle.succeed if ok else goal_handle.abort)()
            self.get_logger().info(f'Resultado: {mensaje}')
            return result

        if goal.target_pose.header.frame_id:
            frame = goal.target_pose.header.frame_id
            target = _pose2_of_msg(goal.target_pose.pose)
            slot_tol = (0.0, 0.0)
            nombre = f'pose en {frame}'
        elif goal.slot_id in self.slots:
            slot = self.slots[goal.slot_id]
            frame, target = slot['frame'], slot['pose']
            slot_tol = (slot['tol_xy'], slot['tol_yaw'])
            nombre = goal.slot_id
        else:
            return terminar(False, f'slot desconocido "{goal.slot_id}"; disponibles: '
                                   f'{sorted(self.slots)}')
        tol_xy = resolve_tolerance(goal.tolerance_xy, slot_tol[0],
                                   self.get_parameter('default_tolerance_xy').value)
        tol_yaw = resolve_tolerance(goal.tolerance_yaw, slot_tol[1], math.radians(
            self.get_parameter('default_tolerance_yaw_deg').value))
        self.get_logger().info(f'Meta: {nombre} {target} ± {tol_xy:.3f} m / '
                               f'{math.degrees(tol_yaw):.1f}°')

        intentos = 1 + max(0, self.get_parameter('max_retries').value)
        for intento in range(1, intentos + 1):
            if self.use_nav2:
                ok, mensaje = self._navegar(goal_handle, frame, target)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.status_message = 'cancelada'
                    return result
                if not ok:
                    return terminar(False, mensaje)
            time.sleep(self.get_parameter('settle_time_s').value)
            fb = PrepareDeliveryPose.Feedback(state='VERIFYING')
            goal_handle.publish_feedback(fb)
            actual = self._car_pose(frame)
            if actual is None:
                return terminar(False, f'no hay TF {frame} -> {self.car_frame}: ¿se ve el '
                                       'tag del carrito?')
            err = pose_error(target, actual)
            if err[0] <= tol_xy and abs(err[1]) <= tol_yaw:
                return terminar(True, f'{nombre} alcanzado (intento {intento}): '
                                      f'{1000 * err[0]:.0f} mm, {math.degrees(err[1]):+.1f}°',
                                frame, actual, err)
            self.get_logger().warn(f'Fuera de tolerancia ({1000 * err[0]:.0f} mm, '
                                   f'{math.degrees(err[1]):+.1f}°), intento {intento}')
            if not self.use_nav2:
                break
        return terminar(False, f'fuera de tolerancia: {1000 * err[0]:.0f} mm, '
                               f'{math.degrees(err[1]):+.1f}°', frame, actual, err)

    def _navegar(self, goal_handle, frame, target):
        """Enviar la meta a Nav2 y esperar; devuelve (éxito, mensaje)."""
        try:
            en_nav = compose(self._lookup2(self.nav_frame, frame), target) \
                if frame != self.nav_frame else target
        except TransformException as error:
            return False, f'no se puede expresar la meta en {self.nav_frame}: {error}'
        if not self.nav.wait_for_server(timeout_sec=5.0):
            return False, 'Nav2 no responde (navigate_to_pose); ¿está lanzado?'

        meta = NavigateToPose.Goal()
        meta.pose = _pose_msg(self.nav_frame, en_nav, self.get_clock().now().to_msg())

        def al_feedback(msg):
            fb = PrepareDeliveryPose.Feedback()
            fb.state = 'NAVIGATING'
            fb.distance_remaining = float(msg.feedback.distance_remaining)
            fb.current_pose = msg.feedback.current_pose
            goal_handle.publish_feedback(fb)

        listo = threading.Event()
        envio = self.nav.send_goal_async(meta, feedback_callback=al_feedback)
        envio.add_done_callback(lambda _: listo.set())
        if not listo.wait(10.0) or not envio.result().accepted:
            return False, 'Nav2 rechazó la meta'
        handle = envio.result()
        fin = threading.Event()
        futuro = handle.get_result_async()
        futuro.add_done_callback(lambda _: fin.set())
        limite = time.monotonic() + self.get_parameter('nav2_timeout_s').value
        while not fin.wait(0.1):
            if goal_handle.is_cancel_requested or time.monotonic() > limite:
                handle.cancel_goal_async()
                return False, ('cancelada' if goal_handle.is_cancel_requested
                               else 'Nav2 no llegó a tiempo')
        estado = futuro.result().status
        if estado != GoalStatus.STATUS_SUCCEEDED:
            return False, f'Nav2 terminó con estado {estado}'
        return True, 'Nav2 llegó'


def main(args=None):
    """Punto de entrada del ejecutable delivery_pose_server."""
    rclpy.init(args=args)
    node = DeliveryPoseServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
