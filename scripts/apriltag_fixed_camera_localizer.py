#!/usr/bin/env python3
"""
apriltag_fixed_camera_localizer.py
Nodo ROS 2 de localización visual 2D con cámara en posición fija (sin requerir TF2).

Publica la pose 2D (x, y, theta) de un carrito en geometry_msgs/Pose2D para módulos
micro-ROS, en uno de dos modos:

- simulated_mode:=true  -> trayectoria sintética (círculo de 0.35 m), sin cámara.
- simulated_mode:=false -> detecta AprilTags (36h11) en la imagen comprimida de la
  cámara del Kinova (la publica la estación anfitriona) y expresa la pose del tag del
  carrito en el plano del tag de la mesa (reference_tag_id), usando la homografía
  definida por las 4 esquinas de ese tag y su tamaño físico (tag_size_m).
  Con reference_tag_id < 0 usa el modelo simple: origen en el centro de la imagen y
  escala fija pixels_per_meter.
"""

import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import CompressedImage

try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


def _make_detector():
    """Detector AprilTag 36h11 compatible con OpenCV 4.6 (Ubuntu 24.04) y 4.7+."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'ArucoDetector'):          # OpenCV >= 4.7
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        return detector.detectMarkers
    parameters = cv2.aruco.DetectorParameters_create()  # OpenCV 4.6
    return lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)


class FixedCameraLocalizer(Node):

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

        self.robot_ns = self.get_parameter('robot_namespace').value
        self.target_tag_id = self.get_parameter('tag_id').value
        self.reference_tag_id = self.get_parameter('reference_tag_id').value
        self.tag_size_m = self.get_parameter('tag_size_m').value
        self.rate_hz = self.get_parameter('publish_rate_hz').value
        self.ppm = self.get_parameter('pixels_per_meter').value
        self.image_topic = self.get_parameter('image_topic').value
        self.simulated_mode = self.get_parameter('simulated_mode').value

        # Publicador de Pose2D para el cliente micro-ROS (ESP32)
        topic_name = f'/{self.robot_ns}/pose2d'
        self.pub_pose2d = self.create_publisher(Pose2D, topic_name, 10)

        self.sim_time = 0.0
        self.detect = None

        if self.simulated_mode:
            self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_callback)
            source = f'trayectoria simulada a {self.rate_hz} Hz'
        else:
            if not CV2_AVAILABLE:
                raise RuntimeError('Modo real requiere OpenCV y NumPy: sudo apt install python3-opencv')
            if self.reference_tag_id >= 0 and self.tag_size_m <= 0.0:
                raise RuntimeError('Con reference_tag_id >= 0 hay que indicar tag_size_m (> 0, en metros)')
            self.detect = _make_detector()
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
        # Trayectoria circular suave para pruebas sin hardware
        msg = Pose2D()
        self.sim_time += 0.05
        radius = 0.35  # 35 cm de radio
        msg.x = float(radius * math.cos(self.sim_time))
        msg.y = float(radius * math.sin(self.sim_time))
        # Ángulo tangente a la trayectoria, normalizado a [-pi, pi]
        msg.theta = float(self.sim_time + math.pi / 2.0)
        msg.theta = math.atan2(math.sin(msg.theta), math.cos(msg.theta))
        self.pub_pose2d.publish(msg)

    def image_callback(self, msg: CompressedImage):
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

        # Centro del tag y punto medio de su borde derecho (eje +x del tag)
        center = points.mean(axis=0)
        forward = (points[1] + points[2]) / 2.0

        msg = Pose2D()
        msg.x = float(center[0])
        msg.y = float(center[1])
        msg.theta = float(math.atan2(forward[1] - center[1], forward[0] - center[0]))
        self.pub_pose2d.publish(msg)

    def _to_reference_plane(self, reference, target):
        """Homografía imagen -> plano del tag_mesa (metros, x a la derecha del tag, y hacia arriba)."""
        h = self.tag_size_m / 2.0
        reference_m = np.array([[-h, h], [h, h], [h, -h], [-h, -h]], dtype=np.float32)
        homography = cv2.getPerspectiveTransform(reference.astype(np.float32), reference_m)
        return cv2.perspectiveTransform(target.reshape(1, 4, 2).astype(np.float32), homography)[0]

    def _to_image_center(self, shape, target):
        """Modelo simple: origen en el centro de la imagen, escala fija, eje y invertido."""
        rows, cols = shape[:2]
        x = (target[:, 0] - cols / 2.0) / self.ppm
        y = -(target[:, 1] - rows / 2.0) / self.ppm
        return np.stack([x, y], axis=1)


def main(args=None):
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
