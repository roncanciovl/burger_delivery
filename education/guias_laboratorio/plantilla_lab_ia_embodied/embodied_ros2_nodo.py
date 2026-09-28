#!/usr/bin/env python3
"""
Opción B — Embodied AI CON ROS 2: nodo que consume el driver kinova_vision y consulta
a Gemini Robotics-ER 2.

La estación anfitriona lanza la cámara (una sola vez para todo el laboratorio):

    ros2 launch kinova_vision kinova_vision.launch.py device:=192.168.1.10 \\
        max_color_pub_rate:=10.0 max_depth_pub_rate:=5.0

Cada grupo ejecuta el nodo (mismo ROS_DOMAIN_ID que la anfitriona):

    export GEMINI_API_KEY="..."
    python3 embodied_ros2_nodo.py --ros-args -p grupo:=G03 -p thinking:=low

Y le hace preguntas publicando en /embodied/consulta (JSON o texto plano):

    ros2 topic pub --once /embodied/consulta std_msgs/msg/String \\
        "{data: '{\\"tarea\\": \\"puntos\\", \\"objeto\\": \\"la caja de hamburguesa\\"}'}"

Para guardar la escena actual (mismo formato que ``embodied_sin_ros.py capturar``):

    ros2 topic pub --once /embodied/consulta std_msgs/msg/String \\
        "{data: '{\\"tarea\\": \\"guardar\\", \\"escena\\": \\"semantica1\\"}'}"

Publica:
    /embodied/respuesta  std_msgs/String        JSON con píxeles, xyz y latencia
    /embodied/objetivo   geometry_msgs/PointStamped  primer objeto, marco óptico de color
    /embodied/anotada/compressed  sensor_msgs/CompressedImage  imagen con el resultado
    TF  <frame de color> -> objetivo_gemini_<i>

No usa cv_bridge: la imagen comprimida se decodifica con OpenCV y la profundidad
(16UC1, milímetros) se lee directamente del búfer del mensaje.
"""

from __future__ import annotations

import json
import threading
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped  # noqa: F401 (TODO 5)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

import embodied_comun as ec

AQUI = Path(__file__).resolve().parent


def intrinsecos_desde_camera_info(msg: CameraInfo) -> ec.Intrinsecos:
    return ec.Intrinsecos(msg.k[0], msg.k[4], msg.k[2], msg.k[5], msg.width, msg.height,
                          fuente=f"camera_info ({msg.header.frame_id})")


def profundidad_desde_mensaje(msg: Image) -> np.ndarray:
    if msg.encoding not in ("16UC1", "mono16"):
        raise ValueError(f"Profundidad con codificación inesperada: {msg.encoding}")
    fila = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)
    depth = fila[:, : msg.width]
    return depth.byteswap() if msg.is_bigendian else depth


class NodoEmbodied(Node):
    def __init__(self):
        super().__init__("embodied_gemini")
        self.declare_parameter("grupo", "G00")
        self.declare_parameter("modelo", ec.MODELO_POR_DEFECTO)
        self.declare_parameter("thinking", "low")
        self.declare_parameter("temperatura", 0.5)
        self.declare_parameter("registro", True)
        # Intrínsecos: "camera_info" usa los del driver; una ruta usa ese JSON.
        self.declare_parameter("fuente_intrinsecos", str(ec.CONFIG_POR_DEFECTO))
        self.declare_parameter("tema_color", "/camera/color/image_raw/compressed")
        self.declare_parameter("tema_profundidad", "/camera/depth/image_raw")
        self.declare_parameter("csv", str(AQUI / "resultados" / "resultados.csv"))

        self._color = None      # (stamp, frame_id, imagen BGR)
        self._depth = None      # (stamp, imagen uint16 mm)
        self._info_color = None
        self._info_depth = None
        self._ocupado = threading.Lock()
        self._cliente = None

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.create_subscription(CompressedImage, p("tema_color"), self._cb_color, qos_profile_sensor_data)
        self.create_subscription(Image, p("tema_profundidad"), self._cb_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self._cb_info_color, 10)
        self.create_subscription(CameraInfo, "/camera/depth/camera_info", self._cb_info_depth, 10)
        self.create_subscription(String, "/embodied/consulta", self._cb_consulta, 10)

        self._pub_resp = self.create_publisher(String, "/embodied/respuesta", 10)
        self._pub_obj = self.create_publisher(PointStamped, "/embodied/objetivo", 10)
        self._pub_img = self.create_publisher(CompressedImage, "/embodied/anotada/compressed", 1)
        self._tf = TransformBroadcaster(self)
        self.get_logger().info("Listo. Publique una consulta en /embodied/consulta")

    # -- Suscripciones: solo guardan el último mensaje (baratas, nunca bloquean) --
    def _cb_color(self, msg: CompressedImage):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            self._color = (msg.header.stamp, msg.header.frame_id, img)

    def _cb_depth(self, msg: Image):
        self._depth = (msg.header.stamp, profundidad_desde_mensaje(msg))

    def _cb_info_color(self, msg):
        self._info_color = msg

    def _cb_info_depth(self, msg):
        self._info_depth = msg

    def _cb_consulta(self, msg: String):
        try:
            pedido = json.loads(msg.data)
        except json.JSONDecodeError:
            pedido = {"tarea": "puntos", "objeto": msg.data}
        if self._color is None:
            self.get_logger().warn("Aún no llega ninguna imagen de color: ¿está activo el driver?")
            return
        if pedido.get("tarea") == "guardar":
            self._guardar_escena(pedido.get("escena") or datetime.now().strftime("escena_%H%M%S"),
                                 pedido.get("nota", ""))
            return
        if not self._ocupado.acquire(blocking=False):
            self.get_logger().warn("Hay una consulta en curso; se descarta la nueva.")
            return
        # La llamada a Gemini tarda segundos: si se hiciera aquí, el executor dejaría
        # de atender las demás suscripciones. Por eso corre en un hilo aparte.
        foto = (self._color, self._depth)
        threading.Thread(target=self._trabajar, args=(pedido, foto), daemon=True).start()

    def _guardar_escena(self, nombre: str, nota: str):
        """Guardar la última imagen de color y de profundidad con el mismo formato que
        ``embodied_sin_ros.py capturar``, para analizarlas después sin el robot."""
        carpeta = AQUI / "capturas"
        carpeta.mkdir(parents=True, exist_ok=True)
        base = carpeta / nombre
        stamp_c, _, color = self._color
        cv2.imwrite(f"{base}_color.png", color)
        meta = {"escena": nombre, "fuente": "ROS 2 (embodied_ros2_nodo.py)",
                "fecha": datetime.now().isoformat(timespec="seconds"),
                "color": {"ancho": color.shape[1], "alto": color.shape[0]}}
        if self._depth is not None:
            stamp_d, depth = self._depth
            cv2.imwrite(f"{base}_depth.png", depth)
            validos = depth[depth > 0]
            meta["profundidad"] = {"ancho": depth.shape[1], "alto": depth.shape[0], "unidad": "mm",
                                   "pixeles_validos_pct": round(100 * validos.size / depth.size, 1),
                                   "mediana_mm": int(np.median(validos)) if validos.size else None}
            dt = (stamp_c.sec - stamp_d.sec) + (stamp_c.nanosec - stamp_d.nanosec) * 1e-9
            meta["desfase_color_profundidad_ms"] = round(abs(dt) * 1000, 1)
        if nota:
            meta["nota"] = nota
        Path(f"{base}_meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))
        self.get_logger().info(f"Escena guardada en {base}_*.png")

    # -- Trabajo pesado, fuera del hilo del executor --
    def _config(self, ancho: int, alto: int) -> ec.ConfigCamara:
        fuente = self.get_parameter("fuente_intrinsecos").value
        if fuente == "camera_info":
            if self._info_color is None or self._info_depth is None:
                raise RuntimeError("No han llegado los camera_info de color y profundidad")
            base = ec.cargar_config(None)
            return ec.ConfigCamara(intrinsecos_desde_camera_info(self._info_color),
                                   intrinsecos_desde_camera_info(self._info_depth),
                                   base.rotacion_depth_a_color, base.traslacion_depth_a_color)
        return ec.cargar_config(fuente, ancho, alto)

    def _trabajar(self, pedido: dict, foto):
        try:
            (stamp, frame_id, color), depth_msg = foto
            depth = depth_msg[1] if depth_msg else None
            tarea = pedido.get("tarea", "puntos")
            objeto = pedido.get("objeto", "el objeto principal")
            cfg = self._config(color.shape[1], color.shape[0])
            for aviso in ec.advertencias_intrinsecos(cfg.color):
                self.get_logger().warn(f"Intrínsecos de color sospechosos: {aviso}")

            if self._cliente is None:
                self._cliente = ec.crear_cliente()
            r = ec.consultar_gemini(self._cliente, [color], ec.construir_prompt(tarea, objeto),
                                    modelo=self.get_parameter("modelo").value,
                                    thinking=self.get_parameter("thinking").value,
                                    temperatura=self.get_parameter("temperatura").value)
            datos = ec.extraer_json(r["texto"])
            if tarea == "plan" and isinstance(datos, dict):
                datos = datos.get("objetos", [])
            localizados = ec.localizar(ec.detecciones_desde_respuesta(datos), color, depth, cfg,
                                       registrar=self.get_parameter("registro").value)

            self._publicar(stamp, frame_id, localizados)
            anotada = ec.dibujar(color, localizados)
            ok, buf = cv2.imencode(".jpg", anotada)
            if ok:
                img = CompressedImage(format="jpeg", data=buf.tobytes())
                img.header.stamp, img.header.frame_id = stamp, frame_id
                self._pub_img.publish(img)
            self._pub_resp.publish(String(data=json.dumps(
                {"tarea": tarea, "objeto": objeto, "latencia_s": r["latencia_s"], "objetos": localizados},
                ensure_ascii=False)))
            self._guardar_csv(tarea, objeto, r, localizados)
            self.get_logger().info(f"{len(localizados)} objeto(s) en {r['latencia_s']} s: "
                                   + ", ".join(f"{d['label']}={d.get('xyz_m')}" for d in localizados))
        except Exception as e:  # noqa: BLE001 — un fallo de red no debe tumbar el nodo
            self.get_logger().error(f"Consulta fallida: {type(e).__name__}: {e}")
            self._pub_resp.publish(String(data=json.dumps({"error": str(e)}, ensure_ascii=False)))
        finally:
            self._ocupado.release()

    def _publicar(self, stamp, frame_id: str, localizados: list[dict]):
        """Publicar el punto 3D como PointStamped y como TF.

        Los puntos están en la convención ÓPTICA (X derecha, Y abajo, Z adelante),
        referidos al frame de la imagen de color (``frame_id`` del mensaje).
        """
        transformadas = []
        for i, det in enumerate(localizados):
            if not det.get("xyz_m"):
                continue
            x, y, z = det["xyz_m"]
            # TODO 5 (solo Opción B): construya el PointStamped (solo para i == 0)
            # y un TransformStamped por objeto con child_frame_id=f"objetivo_gemini_{i}".
            raise NotImplementedError("TODO 5: complete esta función (ver la guía)")
        if transformadas:
            self._tf.sendTransform(transformadas)

    def _guardar_csv(self, tarea, objeto, r, localizados):
        comunes = {"fecha": datetime.now().isoformat(timespec="seconds"),
                   "grupo": self.get_parameter("grupo").value, "opcion": "B-ROS2", "escena": "en_vivo",
                   "tarea": tarea, "objeto": objeto, "modelo": self.get_parameter("modelo").value,
                   "thinking": self.get_parameter("thinking").value,
                   "registro": "si" if self.get_parameter("registro").value else "no",
                   "latencia_s": r["latencia_s"], "tokens_entrada": r.get("tokens_entrada"),
                   "tokens_salida": r.get("tokens_salida"), "tokens_pensamiento": r.get("tokens_pensamiento"),
                   "respuesta": r["texto"].replace("\n", " ")}
        for det in localizados or [{}]:
            xyz = det.get("xyz_m") or [None, None, None]
            ec.guardar_fila_csv(self.get_parameter("csv").value, {
                **comunes, "label": det.get("label"), "punto_yx": det.get("point"), "u": det.get("u"),
                "v": det.get("v"), "z_m": det.get("z_m"), "x_m": xyz[0], "y_m": xyz[1],
                "rango_m": det.get("rango_m")})


def main():
    rclpy.init()
    nodo = NodoEmbodied()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
