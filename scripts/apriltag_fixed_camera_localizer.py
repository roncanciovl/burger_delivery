#!/usr/bin/env python3
"""
apriltag_fixed_camera_localizer.py
Nodo ROS 2 de localización visual con cámara en posición fija (sin requerir TF2).
Detecta marcadores AprilTag / ArUco en la imagen, calcula su pose 2D (x, y, theta)
en el plano de la mesa y la publica en geometry_msgs/Pose2D para módulos micro-ROS.
"""

import math
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


class FixedCameraLocalizer(Node):

    def __init__(self):
        super().__init__('apriltag_fixed_camera_localizer')

        # Parámetros del nodo
        self.declare_parameter('robot_namespace', 'burger_car_01')
        self.declare_parameter('tag_id', 0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('pixels_per_meter', 500.0)  # Factor de escala visual
        self.declare_parameter('simulated_mode', False)

        self.robot_ns = self.get_parameter('robot_namespace').value
        self.target_tag_id = self.get_parameter('tag_id').value
        self.rate_hz = self.get_parameter('publish_rate_hz').value
        self.ppm = self.get_parameter('pixels_per_meter').value
        self.simulated_mode = self.get_parameter('simulated_mode').value

        # Publicador de Pose2D para el cliente micro-ROS (ESP32)
        topic_name = f'/{self.robot_ns}/pose2d'
        self.pub_pose2d = self.create_publisher(Pose2D, topic_name, 10)

        # Variables para simulación o detección
        self.sim_time = 0.0

        # Configurar detector ArUco / AprilTag con OpenCV si está disponible
        self.detector = None
        if CV2_AVAILABLE:
            try:
                # Diccionario ArUco compatible con AprilTags (DICT_APRILTAG_36h11 o DICT_4X4_50)
                dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
                parameters = cv2.aruco.DetectorParameters()
                self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            except AttributeError:
                # Compatibilidad con versiones anteriores de OpenCV
                self.detector = None

        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_callback)

        self.get_logger().info(
            f'📷 Fixed Camera Localizer inicializado (Cámara Fija - Sin TF2).\n'
            f'   Target Tag ID: {self.target_tag_id}\n'
            f'   Publicando en: {topic_name} a {self.rate_hz} Hz\n'
            f'   Modo Simulación: {self.simulated_mode}'
        )

    def timer_callback(self):
        msg = Pose2D()

        if self.simulated_mode:
            # Generar trayectoria circular suave para pruebas sin hardware
            self.sim_time += 0.05
            radius = 0.35  # 35 cm de radio
            msg.x = float(radius * math.cos(self.sim_time))
            msg.y = float(radius * math.sin(self.sim_time))
            # Ángulo tangente a la trayectoria
            msg.theta = float(self.sim_time + math.pi / 2.0)
            # Normalizar theta a [-pi, pi]
            msg.theta = math.atan2(math.sin(msg.theta), math.cos(msg.theta))

            self.pub_pose2d.publish(msg)
            return

        # En modo real, se procesa la detección visual
        # Ejemplo: cuando se integra con captura de cámara en vivo
        pass

    def process_frame(self, frame: 'np.ndarray'):
        """Procesa una imagen y extrae la pose 2D del marcador."""
        if not CV2_AVAILABLE or self.detector is None or frame is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None and self.target_tag_id in ids:
            idx = int(np.where(ids == self.target_tag_id)[0][0])
            c = corners[idx][0]  # 4 esquinas del marcador

            # Centro del tag en píxeles
            center_x = float(np.mean(c[:, 0]))
            center_y = float(np.mean(c[:, 1]))

            # Centro óptico de la imagen como origen (0, 0)
            h, w = frame.shape[:2]
            origin_x = w / 2.0
            origin_y = h / 2.0

            # Conversión a metros en el plano de la mesa
            pos_x_meters = (center_x - origin_x) / self.ppm
            pos_y_meters = -(center_y - origin_y) / self.ppm  # Invertir eje Y visual

            # Cálculo de orientación 2D (Yaw) a partir del vector de la esquina 0 a la 1
            dx = c[1][0] - c[0][0]
            dy = -(c[1][1] - c[0][1])
            yaw = math.atan2(dy, dx)

            msg = Pose2D()
            msg.x = float(pos_x_meters)
            msg.y = float(pos_y_meters)
            msg.theta = float(yaw)

            self.pub_pose2d.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FixedCameraLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
