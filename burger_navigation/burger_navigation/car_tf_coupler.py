# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""
Acople dinámico del carrito al árbol de TF durante el movimiento (TODO.md §3).

Convierte la ``Pose2D`` que publica el localizador AprilTag (``/<ns>/pose2d``, en el plano
del ``tag_mesa``) en el TF ``tag_mesa -> tag_carritoN``. El resto de la cadena,
``tag_carritoN -> carN_base_link``, es fija y la publica el ``robot_state_publisher`` del
carrito (``carN_apriltag.urdf``). Así, mientras el carrito se mueve, ``map -> carN_base_link``
se mantiene al día sin odometría.

Además:

* **Vencimiento**: si no llega pose en ``stale_timeout_s`` deja de publicar (un TF viejo
  republicado con hora nueva haría creer que el carrito sigue ahí) y lo avisa.
* **Suavizado** opcional (``smoothing`` en (0, 1]; 1 = sin filtro).
* **Corrección map -> odom** (``odom_frame`` no vacío): para que Nav2 use la odometría de
  ruedas del carrito con la localización del tag, publica ``map -> odom`` como lo haría
  AMCL. Necesita que el carrito publique ``odom -> base``.
"""

from burger_navigation.delivery_logic import (
    map_to_odom,
    normalize_angle,
    quaternion_from_yaw,
    yaw_from_quaternion,
)
from geometry_msgs.msg import Pose2D, TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class CarTfCoupler(Node):
    """Pose2D del localizador -> TF tag_mesa -> tag_carrito (y map -> odom opcional)."""

    def __init__(self):
        """Declarar parámetros, suscripción y difusor de TF."""
        super().__init__('car_tf_coupler')
        p = self.declare_parameter
        p('pose_topic', '/burger_car_01/pose2d')
        p('reference_frame', 'tag_mesa')
        p('tag_frame', 'tag_carrito1')
        p('tag_height_m', 0.0)
        p('stale_timeout_s', 0.5)
        p('smoothing', 1.0)
        p('rate_hz', 30.0)
        p('odom_frame', '')
        p('base_frame', 'car1_base_link')
        p('map_frame', 'map')

        g = self.get_parameter
        self.reference = g('reference_frame').value
        self.tag_frame = g('tag_frame').value
        self.z = g('tag_height_m').value
        self.timeout = g('stale_timeout_s').value
        self.alpha = min(1.0, max(0.01, g('smoothing').value))
        self.odom_frame = g('odom_frame').value
        self.pose = None
        self.last = None
        self.stale_warned = False

        self.broadcaster = TransformBroadcaster(self)
        if self.odom_frame:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Pose2D, g('pose_topic').value, self._on_pose, 10)
        self.create_timer(1.0 / g('rate_hz').value, self._publish)
        self.get_logger().info(
            f'{g("pose_topic").value} -> TF {self.reference} -> {self.tag_frame}'
            + (f'; corrección {g("map_frame").value} -> {self.odom_frame}'
               if self.odom_frame else ''))

    def _on_pose(self, msg):
        nueva = (msg.x, msg.y, msg.theta)
        if self.pose is None or self.alpha >= 1.0:
            self.pose = nueva
        else:
            a = self.alpha
            dyaw = normalize_angle(nueva[2] - self.pose[2])
            self.pose = (self.pose[0] + a * (nueva[0] - self.pose[0]),
                         self.pose[1] + a * (nueva[1] - self.pose[1]),
                         normalize_angle(self.pose[2] + a * dyaw))
        self.last = self.get_clock().now()
        if self.stale_warned:
            self.get_logger().info('Pose del carrito recuperada')
            self.stale_warned = False

    def _publish(self):
        if self.pose is None:
            return
        ahora = self.get_clock().now()
        if (ahora - self.last) > Duration(seconds=self.timeout):
            if not self.stale_warned:
                self.get_logger().warn(
                    f'Sin pose del carrito hace más de {self.timeout:.1f} s: se deja de '
                    f'publicar {self.tag_frame} (¿tag tapado o fuera de cámara?)')
                self.stale_warned = True
            return
        tf = TransformStamped()
        tf.header.stamp = ahora.to_msg()
        tf.header.frame_id = self.reference
        tf.child_frame_id = self.tag_frame
        tf.transform.translation.x, tf.transform.translation.y = self.pose[0], self.pose[1]
        tf.transform.translation.z = self.z
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = quaternion_from_yaw(self.pose[2])
        transforms = [tf]
        correccion = self._map_to_odom() if self.odom_frame else None
        if correccion is not None:
            transforms.append(correccion)
        self.broadcaster.sendTransform(transforms)

    def _map_to_odom(self):
        g = self.get_parameter
        try:
            mapa = self._lookup2(g('map_frame').value, g('base_frame').value)
            odom = self._lookup2(self.odom_frame, g('base_frame').value)
        except TransformException:
            return None
        x, y, yaw = map_to_odom(mapa, odom)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = g('map_frame').value
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x, tf.transform.translation.y = x, y
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = quaternion_from_yaw(yaw)
        return tf

    def _lookup2(self, parent, child):
        tf = self.tf_buffer.lookup_transform(parent, child, Time())
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)


def main(args=None):
    """Punto de entrada del ejecutable car_tf_coupler."""
    rclpy.init(args=args)
    node = CarTfCoupler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
