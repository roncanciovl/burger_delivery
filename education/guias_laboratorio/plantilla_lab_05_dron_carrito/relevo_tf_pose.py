#!/usr/bin/env python3
"""
Relevo TF -> PoseStamped para el ESP32 del carrito (Laboratorio 05, Fase A).

El equipo del dron publica en /tf la transformación tag_mesa -> tag_carrito1. Este nodo,
en la PC del equipo del carrito, la lee con tf2 y la reenvía como
geometry_msgs/PoseStamped en /<robot_namespace>/pose_mesa, un tópico con un solo
publicador y un solo tipo de mensaje que el ESP32 puede suscribir sin reservar memoria
para todo /tf.

La pose conserva la estampa de la TF (cuándo se vio el carrito), no la de este nodo. Si la
TF más reciente es más vieja que max_age_s, el relevo NO publica: el ESP32 detecta el
silencio con su propio timeout y detiene los motores.

Uso:
    python3 relevo_tf_pose.py --ros-args -p robot_namespace:=burger_car_01 \\
        -p parent_frame:=tag_mesa -p child_frame:=tag_carrito1
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class RelevoTfPose(Node):

    def __init__(self):
        super().__init__('relevo_tf_pose')
        self.declare_parameter('robot_namespace', 'burger_car_01')
        self.declare_parameter('parent_frame', 'tag_mesa')
        self.declare_parameter('child_frame', 'tag_carrito1')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('max_age_s', 0.3)

        ns = self.get_parameter('robot_namespace').value
        self.parent = self.get_parameter('parent_frame').value
        self.child = self.get_parameter('child_frame').value
        self.max_age = Duration(seconds=float(self.get_parameter('max_age_s').value))
        rate_hz = float(self.get_parameter('rate_hz').value)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        topic = f'/{ns}/pose_mesa'
        self.pub = self.create_publisher(PoseStamped, topic, 10)
        self.last_stamp = None
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f'Relevo {self.parent} -> {self.child} => {topic} '
            f'(PoseStamped, {rate_hz} Hz, edad máxima {self.max_age.nanoseconds / 1e9} s)')

    def tick(self):
        try:
            # Time() = la transformación más reciente disponible, sin interpolar
            t = self.buffer.lookup_transform(self.parent, self.child, Time())
        except TransformException as exc:
            self.get_logger().warn(f'Sin TF {self.parent} -> {self.child}: {exc}',
                                   throttle_duration_sec=2.0)
            return

        stamp = Time.from_msg(t.header.stamp)
        age = self.get_clock().now() - stamp
        if age > self.max_age:
            self.get_logger().warn(
                f'TF vieja ({age.nanoseconds / 1e9:.2f} s > {self.max_age.nanoseconds / 1e9} s): '
                'no se publica pose', throttle_duration_sec=2.0)
            return
        if self.last_stamp == stamp:
            return  # misma imagen: no reenviar la pose repetida
        self.last_stamp = stamp

        msg = PoseStamped()
        msg.header = t.header                     # estampa y marco (tag_mesa) de la TF
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.position.z = t.transform.translation.z
        msg.pose.orientation = t.transform.rotation
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RelevoTfPose()
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
