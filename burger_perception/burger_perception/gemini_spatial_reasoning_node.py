# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""
Nodo de razonamiento espacial con Gemini Robotics-ER (TODO.md §1).

Flujo, desacoplado para no bloquear los callbacks de la cámara:

1. **Captura asíncrona**: se guardan la última imagen comprimida, su ``camera_info`` y la
   última profundidad; los callbacks sólo copian referencias.
2. **Inferencia** en un hilo aparte, bajo demanda (servicio ``~/locate``) o cada
   ``inference_period_s`` segundos (0 = sólo bajo demanda: cada llamada cuesta 2-5 s y
   dinero, ver ``ros2_setup/PROPUESTA_GEMINI_ER.md``).
3. **Desproyección** del píxel con ``image_geometry`` y la profundidad (mediana de una
   ventana), en el marco óptico de la cámara.
4. **Publicación**: el punto se expresa en ``fixed_frame`` (``base_link``) con el TF del
   instante de la imagen, porque la cámara del Kinova va en la muñeca y se mueve; y se
   difunde como TF ``fixed_frame -> target_frame`` (``target_burger_box_frame``) a 10 Hz
   hasta la siguiente inferencia. Si no hay TF de la cámara, se publica respecto al marco
   óptico y se avisa.

Métricas de cada inferencia (latencia, bytes enviados, píxel, punto) en ``~/inference``
como JSON, para el benchmark contra AprilTag.

Modos de profundidad (``depth_mode``):

* ``registered``: profundidad alineada con el color (``depth_registration:=true`` en
  ``kinova_vision``); se desproyecta con el ``camera_info`` del color.
* ``scaled``: profundidad sin registrar; el píxel se escala a la imagen de profundidad y se
  desproyecta con su ``camera_info``. Aproximado: ignora la separación entre sensores.
* ``plane``: sin profundidad; se usa ``plane_distance_m`` como z (cámara perpendicular).

La clave va en ``GEMINI_API_KEY``, nunca como parámetro.
"""

import json
import threading

from burger_perception.gemini_client import build_prompt, DEFAULT_MODEL, GeminiPointer
from burger_perception.vlm_geometry import (
    depth_at,
    normalized_to_pixel,
    scale_pixel,
    scale_ray_to_depth,
)
import cv2
from geometry_msgs.msg import PointStamped, TransformStamped
from image_geometry import PinholeCameraModel
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

_DEPTH_DTYPES = {'16UC1': np.uint16, 'mono16': np.uint16, '32FC1': np.float32}


class GeminiSpatialReasoningNode(Node):
    """Localiza un objeto descrito en lenguaje natural y publica su TF 3D."""

    def __init__(self):
        """Declarar parámetros, suscripciones, servicio y el cliente de Gemini."""
        super().__init__('gemini_spatial_reasoning')
        p = self.declare_parameter
        p('image_topic', '/camera/color/image_raw/compressed')
        p('camera_info_topic', '/camera/color/camera_info')
        p('depth_topic', '/camera/depth_registered/image_rect')
        p('depth_info_topic', '/camera/depth/camera_info')
        p('depth_mode', 'registered')
        p('plane_distance_m', 0.5)
        p('depth_window_px', 7)
        p('target', 'cardboard burger box')
        p('context', 'It must be resting on the table, not held by a person.')
        p('model', DEFAULT_MODEL)
        p('thinking_budget', 0)
        p('max_image_side', 1024)
        p('jpeg_quality', 85)
        p('inference_period_s', 0.0)
        p('fixed_frame', 'base_link')
        p('target_frame', 'target_burger_box_frame')
        p('tf_rate_hz', 10.0)

        g = self.get_parameter
        self.depth_mode = g('depth_mode').value
        if self.depth_mode not in ('registered', 'scaled', 'plane'):
            raise ValueError(f'depth_mode inválido: {self.depth_mode}')
        self.prompt = build_prompt(g('target').value, g('context').value)
        self.fixed_frame = g('fixed_frame').value
        self.target_frame = g('target_frame').value

        self.lock = threading.Lock()
        self.image = None
        self.info = None
        self.depth = None
        self.depth_info = None
        self.last_tf = None
        self.busy = threading.Event()

        self.pointer = GeminiPointer(g('model').value, thinking_budget=g('thinking_budget').value)

        self.create_subscription(CompressedImage, g('image_topic').value,
                                 self._on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g('camera_info_topic').value,
                                 self._on_info, 10)
        if self.depth_mode != 'plane':
            self.create_subscription(Image, g('depth_topic').value,
                                     self._on_depth, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, g('depth_info_topic').value,
                                     self._on_depth_info, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_point = self.create_publisher(PointStamped, '~/target_point', 10)
        self.pub_metrics = self.create_publisher(String, '~/inference', 10)

        worker = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, '~/locate', self._on_locate, callback_group=worker)
        period = g('inference_period_s').value
        if period > 0:
            self.create_timer(period, self._periodic, callback_group=worker)
        self.create_timer(1.0 / g('tf_rate_hz').value, self._rebroadcast)

        modo = f'cada {period:.1f} s' if period > 0 else 'ros2 service call <nodo>/locate'
        self.get_logger().info(
            f'Gemini listo: modelo {self.pointer.model}, profundidad {self.depth_mode}, '
            f'TF {self.fixed_frame} -> {self.target_frame}. '
            f'Inferencia: {modo}')

    # --- captura asíncrona -------------------------------------------------------------
    def _on_image(self, msg):
        with self.lock:
            self.image = msg

    def _on_info(self, msg):
        with self.lock:
            self.info = msg

    def _on_depth(self, msg):
        with self.lock:
            self.depth = msg

    def _on_depth_info(self, msg):
        with self.lock:
            self.depth_info = msg

    # --- inferencia ---------------------------------------------------------------------
    def _periodic(self):
        if not self.busy.is_set():
            self.locate()

    def _on_locate(self, request, response):
        del request
        ok, message = self.locate()
        response.success, response.message = ok, message
        return response

    def locate(self):
        """Ejecutar una inferencia completa; devuelve (éxito, mensaje)."""
        with self.lock:
            image, info, depth, depth_info = self.image, self.info, self.depth, self.depth_info
        if image is None or info is None:
            return False, 'sin imagen o sin camera_info todavía'
        if self.depth_mode != 'plane' and depth is None:
            return False, f'sin profundidad (depth_mode={self.depth_mode})'

        self.busy.set()
        try:
            return self._locate(image, info, depth, depth_info)
        except Exception as error:  # la red o el SDK pueden fallar de muchas formas
            self.get_logger().error(f'Inferencia fallida: {error}')
            return False, str(error)
        finally:
            self.busy.clear()

    def _locate(self, image, info, depth, depth_info):
        bgr = cv2.imdecode(np.frombuffer(image.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return False, f'no se pudo decodificar la imagen ({image.format})'
        height, width = bgr.shape[:2]
        jpeg = self._encode_for_model(bgr)
        points, latency_ms, raw = self.pointer.point(jpeg, self.prompt)
        metrics = {'stamp': Time.from_msg(image.header.stamp).nanoseconds * 1e-9,
                   'latency_ms': latency_ms, 'jpeg_bytes': len(jpeg),
                   'image_size': [width, height], 'points': points, 'ok': False}
        if not points:
            metrics['raw'] = raw[:500]
            self._publish_metrics(metrics)
            return False, f'el modelo no señaló el objeto ({latency_ms:.0f} ms)'

        u, v = normalized_to_pixel(points[0], width, height)
        xyz, frame = self._deproject(u, v, (width, height), info, depth, depth_info)
        metrics.update({'pixel': [u, v]})
        if xyz is None:
            self._publish_metrics(metrics)
            return False, f'sin profundidad válida en ({u:.0f}, {v:.0f})'

        point = PointStamped()
        point.header.stamp = image.header.stamp
        point.header.frame_id = frame
        point.point.x, point.point.y, point.point.z = xyz
        point = self._to_fixed_frame(point)
        self.pub_point.publish(point)
        self._set_target_tf(point)
        metrics.update({'ok': True, 'frame': point.header.frame_id,
                        'xyz': [point.point.x, point.point.y, point.point.z]})
        self._publish_metrics(metrics)
        return True, (f'{points[0]["label"] or "objeto"} en {point.header.frame_id} '
                      f'({point.point.x:.3f}, {point.point.y:.3f}, {point.point.z:.3f}) m, '
                      f'{latency_ms:.0f} ms')

    def _encode_for_model(self, bgr):
        side = self.get_parameter('max_image_side').value
        scale = min(1.0, side / max(bgr.shape[:2]))
        if scale < 1.0:
            bgr = cv2.resize(bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        quality = int(self.get_parameter('jpeg_quality').value)
        ok, data = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise RuntimeError('no se pudo recodificar la imagen')
        return data.tobytes()

    def _deproject(self, u, v, color_size, info, depth, depth_info):
        """Devolver ((x, y, z), frame_id) en el marco óptico, o (None, None)."""
        window = self.get_parameter('depth_window_px').value
        if self.depth_mode == 'plane':
            model, pu, pv = self._model(info), u, v
            z = self.get_parameter('plane_distance_m').value
            frame = info.header.frame_id
        elif self.depth_mode == 'registered':
            model, frame = self._model(info), info.header.frame_id
            array = self._depth_array(depth)
            pu, pv = scale_pixel(u, v, color_size, (array.shape[1], array.shape[0]))
            z = depth_at(array, pu, pv, window, depth.encoding)
            pu, pv = u, v
        else:
            if depth_info is None:
                return None, None
            model, frame = self._model(depth_info), depth_info.header.frame_id
            array = self._depth_array(depth)
            pu, pv = scale_pixel(u, v, color_size, (array.shape[1], array.shape[0]))
            z = depth_at(array, pu, pv, window, depth.encoding)
        if z is None:
            return None, None
        ray = model.projectPixelTo3dRay((pu, pv))
        return scale_ray_to_depth(ray, z), frame

    @staticmethod
    def _model(info):
        model = PinholeCameraModel()
        model.fromCameraInfo(info)
        return model

    @staticmethod
    def _depth_array(msg):
        dtype = _DEPTH_DTYPES.get(msg.encoding)
        if dtype is None:
            raise RuntimeError(f'codificación de profundidad no soportada: {msg.encoding}')
        array = np.frombuffer(msg.data, dtype=dtype)
        return array.reshape(msg.height, msg.step // np.dtype(dtype).itemsize)[:, :msg.width]

    def _to_fixed_frame(self, point):
        """Expresar el punto en fixed_frame con el TF del instante de la imagen."""
        if not self.fixed_frame or point.header.frame_id == self.fixed_frame:
            return point
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame, point.header.frame_id, Time.from_msg(point.header.stamp),
                timeout=Duration(seconds=0.5))
            return do_transform_point(point, tf)
        except TransformException as error:
            self.get_logger().warn(
                f'Sin TF {self.fixed_frame} <- {point.header.frame_id} ({error}); el objetivo '
                'se publica respecto a la cámara y se moverá con ella', throttle_duration_sec=10)
            return point

    def _set_target_tf(self, point):
        tf = TransformStamped()
        tf.header.frame_id = point.header.frame_id
        tf.child_frame_id = self.target_frame
        tf.transform.translation.x = point.point.x
        tf.transform.translation.y = point.point.y
        tf.transform.translation.z = point.point.z
        tf.transform.rotation.w = 1.0
        with self.lock:
            self.last_tf = tf
        self._rebroadcast()

    def _rebroadcast(self):
        with self.lock:
            tf = self.last_tf
        if tf is not None:
            tf.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(tf)

    def _publish_metrics(self, metrics):
        self.pub_metrics.publish(String(data=json.dumps(metrics)))
        level = self.get_logger().info if metrics['ok'] else self.get_logger().warn
        level(f'Inferencia: ok={metrics["ok"]}, {metrics["latency_ms"]:.0f} ms, '
              f'{metrics["jpeg_bytes"] / 1024:.0f} KB enviados')


def main(args=None):
    """Punto de entrada del ejecutable gemini_spatial_reasoning_node."""
    rclpy.init(args=args)
    node = GeminiSpatialReasoningNode()
    executor = MultiThreadedExecutor(num_threads=3)
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
