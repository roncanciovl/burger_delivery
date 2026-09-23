# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""
Nodo ROS 2 de localización visual 2D con cámara en posición fija.

Antes ``scripts/apriltag_fixed_camera_localizer.py``; ese archivo queda como envoltorio
para que los talleres sigan funcionando. Ejecutable del paquete: ``apriltag_localizer``.

Publica la pose 2D (x, y, theta) de un carrito en geometry_msgs/Pose2D para módulos
micro-ROS, en uno de dos modos:

- simulated_mode:=true  -> trayectoria sintética (círculo de 0.35 m), sin cámara.
- simulated_mode:=false -> detecta AprilTags (36h11) en la imagen comprimida de la
  cámara del Kinova (la publica la estación anfitriona) y expresa la pose del tag del
  carrito en el plano del tag de la mesa (reference_tag_id), usando la homografía
  definida por las 4 esquinas de ese tag y su tamaño físico (tag_size_m).
  Con reference_tag_id < 0 usa el modelo simple: origen en el centro de la imagen y
  escala fija pixels_per_meter.

Con publish_tf:=true publica además el TF tag_mesa -> <child_frame> (por defecto
tag_carrito1), que es el eslabón que burger_description espera del localizador y que hoy
sustituye use_static_carts:=true (vision_setup/LOCALIZACION_APRILTAG.md §1).
"""

import math

from burger_perception.tag_geometry import (
    make_detector,
    pose_from_corners,
    to_image_center,
    to_reference_plane,
)
import cv2
from geometry_msgs.msg import Pose2D, TransformStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class FixedCameraLocalizer(Node):
    """Localizador AprilTag con cámara fija; publica Pose2D y, opcionalmente, TF."""

    def __init__(self):
        super().__init__('apriltag_fixed_camera_localizer')

        # Parámetros del nodo
        self.declare_parameter('robot_namespace', 'burger_car_01')
        self.declare_parameter('tag_id', 0)                  # tag del carrito
        self.declare_parameter('reference_tag_id', -1)       # tag_mesa; -1 = sin referencia
        self.declare_parameter('tag_size_m', 0.0)            # lado del cuadro negro del tag_mesa
        self.declare_parameter('publish_rate_hz', 10.0)      # sólo modo simulado
        self.declare_parameter('pixels_per_meter', 500.0)    # sólo sin referencia
        self.declare_parameter('image_topic', '/camera/color/image_raw/compressed')
        self.declare_parameter('simulated_mode', False)
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('reference_frame', 'tag_mesa')
        self.declare_parameter('child_frame', 'tag_carrito1')

        self.robot_ns = self.get_parameter('robot_namespace').value
        self.target_tag_id = self.get_parameter('tag_id').value
        self.reference_tag_id = self.get_parameter('reference_tag_id').value
        self.tag_size_m = self.get_parameter('tag_size_m').value
        self.rate_hz = self.get_parameter('publish_rate_hz').value
        self.ppm = self.get_parameter('pixels_per_meter').value
        self.image_topic = self.get_parameter('image_topic').value
        self.simulated_mode = self.get_parameter('simulated_mode').value
        self.tf_broadcaster = None
        if self.get_parameter('publish_tf').value:
            from tf2_ros import TransformBroadcaster
            self.tf_broadcaster = TransformBroadcaster(self)
            self.reference_frame = self.get_parameter('reference_frame').value
            self.child_frame = self.get_parameter('child_frame').value

        # Publicador de Pose2D para el cliente micro-ROS (ESP32)
        topic_name = f'/{self.robot_ns}/pose2d'
        self.pub_pose2d = self.create_publisher(Pose2D, topic_name, 10)

        self.sim_time = 0.0
        self.detect = None

        if self.simulated_mode:
            self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_callback)
            source = f'trayectoria simulada a {self.rate_hz} Hz'
        else:
            if self.reference_tag_id >= 0 and self.tag_size_m <= 0.0:
                raise RuntimeError(
                    'Con reference_tag_id >= 0 hay que indicar tag_size_m (> 0, en metros)')
            self.detect = make_detector()
            # Perfil de datos de sensor: compatible con publicadores fiables o best effort
            self.sub_image = self.create_subscription(
                CompressedImage, self.image_topic, self.image_callback, qos_profile_sensor_data)
            reference = (f'tag_mesa ID {self.reference_tag_id} ({self.tag_size_m} m)'
                         if self.reference_tag_id >= 0 else
                         f'centro de la imagen, {self.ppm} px/m')
            source = f'imagen {self.image_topic}; referencia: {reference}'

        self.get_logger().info(
            f'📷 Fixed Camera Localizer inicializado (Cámara Fija - Sin TF2).\n'
            f'   Tag del carrito: {self.target_tag_id}\n'
            f'   Publicando en: {topic_name}\n'
            f'   Fuente: {source}\n'
            f'   Modo Simulación: {self.simulated_mode}'
        )

    def timer_callback(self):
        """Publicar una trayectoria circular suave para pruebas sin hardware."""
        msg = Pose2D()
        self.sim_time += 0.05
        radius = 0.35  # 35 cm de radio
        msg.x = float(radius * math.cos(self.sim_time))
        msg.y = float(radius * math.sin(self.sim_time))
        # Ángulo tangente a la trayectoria, normalizado a [-pi, pi]
        msg.theta = float(self.sim_time + math.pi / 2.0)
        msg.theta = math.atan2(math.sin(msg.theta), math.cos(msg.theta))
        self.publish(msg)

    def image_callback(self, msg: CompressedImage):
        """Decodificar la imagen comprimida y procesarla."""
        buffer = np.frombuffer(msg.data, dtype=np.uint8)
        gray = cv2.imdecode(buffer, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            self.get_logger().warn(
                f'No se pudo decodificar la imagen (format="{msg.format}")',
                throttle_duration_sec=5.0)
            return
        self.process_frame(gray)

    def process_frame(self, gray: 'np.ndarray'):
        """Detecta los tags en una imagen en escala de grises y publica la pose 2D."""
        corners, ids, _ = self.detect(gray)
        seen = [] if ids is None else [int(i) for i in ids.ravel()]
        self.get_logger().info(f'Tags visibles: {sorted(seen)}', throttle_duration_sec=5.0)

        if self.target_tag_id not in seen:
            self.get_logger().warn(
                f'Tag del carrito {self.target_tag_id} no visible: no se publica pose',
                throttle_duration_sec=2.0)
            return
        # Esquinas en orden: 0 sup-izq, 1 sup-der, 2 inf-der, 3 inf-izq (píxeles)
        target = corners[seen.index(self.target_tag_id)].reshape(4, 2)

        if self.reference_tag_id >= 0:
            if self.reference_tag_id not in seen:
                self.get_logger().warn(
                    f'Tag de referencia {self.reference_tag_id} no visible: no se publica pose',
                    throttle_duration_sec=2.0)
                return
            reference = corners[seen.index(self.reference_tag_id)].reshape(4, 2)
            points = self._to_reference_plane(reference, target)
        else:
            points = self._to_image_center(gray.shape, target)

        x, y, theta = pose_from_corners(points)
        msg = Pose2D()
        msg.x, msg.y, msg.theta = x, y, theta
        self.publish(msg)

    def _to_reference_plane(self, reference, target):
        return to_reference_plane(reference, target, self.tag_size_m)

    def _to_image_center(self, shape, target):
        return to_image_center(shape, target, self.ppm)

    def publish(self, msg: Pose2D):
        """Publicar la pose 2D y, si se pidió, el TF equivalente en el plano de la mesa."""
        self.pub_pose2d.publish(msg)
        if self.tf_broadcaster is None:
            return
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.reference_frame
        tf.child_frame_id = self.child_frame
        tf.transform.translation.x = msg.x
        tf.transform.translation.y = msg.y
        tf.transform.rotation.z = math.sin(msg.theta / 2.0)
        tf.transform.rotation.w = math.cos(msg.theta / 2.0)
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    """Punto de entrada del ejecutable apriltag_localizer."""
    rclpy.init(args=args)
    node = FixedCameraLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Con Ctrl+C, un callback de imagen puede publicar justo cuando el contexto
        # ya se apagó; sólo en ese caso el error es esperable.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        # En Jazzy, Ctrl+C ya apaga el contexto: un segundo shutdown lanza RCLError.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
