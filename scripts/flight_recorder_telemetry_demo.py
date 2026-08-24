#!/usr/bin/env python3
"""
flight_recorder_telemetry_demo.py
Nodo de telemetría y diagnóstico para el taller de Logging y rosbag2 en ROS 2 Jazzy,
basado en el entorno de Proyecto 1 (Kinova Gen3).

Demuestra:
1. Niveles de log en ROS 2 (DEBUG, INFO, WARN, ERROR, FATAL) con formateo y throttling.
2. Cambio dinámico de nivel de log en tiempo de ejecución vía parámetros/servicios.
3. Publicación de estados articulares (joint_states), telemetría de vibración y diagnóstico del Kinova.
4. Inyección controlada de anomalías (jittering excesivo en articulaciones).
5. Integración con rosbag2 (grabación continua o buffer post-mortem / flight recorder).

Uso:
    python3 scripts/flight_recorder_telemetry_demo.py --ros-args --log-level INFO
"""

import math
import sys
import time
from collections import deque
from typing import Deque, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import Trigger, SetBool


class FlightRecorderTelemetryDemo(Node):
    """
    Nodo emulador de subsistemas del robot con capacidades avanzadas de logging y telemetría.
    """

    def __init__(self):
        super().__init__('flight_recorder_telemetry_demo')

        # Declaración de parámetros configurables dinámicamente
        self.declare_parameter('namespace', 'burger/kinova')
        self.declare_parameter('telemetry_rate_hz', 20.0)
        self.declare_parameter('enable_flight_buffer', True)
        self.declare_parameter('buffer_max_samples', 200)

        # Lectura de parámetros
        self.ns = self.get_parameter('namespace').get_parameter_value().string_value
        rate_hz = self.get_parameter('telemetry_rate_hz').get_parameter_value().double_value
        self.enable_buffer = self.get_parameter('enable_flight_buffer').get_parameter_value().bool_value
        self.max_samples = self.get_parameter('buffer_max_samples').get_parameter_value().integer_value

        # Publicadores de telemetría
        self.pub_joint_states = self.create_publisher(JointState, f'/{self.ns}/joint_states', 10)
        self.pub_jitter = self.create_publisher(Float32, f'/{self.ns}/joint_jitter', 10)
        self.pub_status = self.create_publisher(String, f'/{self.ns}/system_health', 10)

        # Servicios para inyección de anomalías y disparo de blackbox
        self.srv_fault = self.create_service(
            SetBool, f'/{self.ns}/trigger_anomaly', self.trigger_anomaly_callback
        )
        self.srv_dump_buffer = self.create_service(
            Trigger, f'/{self.ns}/dump_flight_recorder', self.dump_flight_recorder_callback
        )

        # Estado cinemático interno del Kinova
        self.joint_names = [f"joint_{i}" for i in range(1, 8)]
        self.joint_positions = [0.0] * 7

        # Control de anomalías
        self.anomaly_active = False
        self.anomaly_type = "NONE"

        # Ring Buffer en memoria (Flight Recorder / Caja Negra)
        self.flight_buffer: Deque[Dict[str, Any]] = deque(maxlen=self.max_samples)

        # Timers
        self.timer_period = 1.0 / max(1.0, rate_hz)
        self.timer = self.create_timer(self.timer_period, self.telemetry_loop)
        self.start_time = time.time()
        self.last_log_time = time.time()

        self.get_logger().info(
            f"🚀 [INIT] Nodo '{self.get_name()}' iniciado para namespace '{self.ns}' a {rate_hz} Hz."
        )
        self.get_logger().info(
            "💡 TIP: Usa 'ros2 param set /flight_recorder_telemetry_demo log_level DEBUG' para ver trazas profundas."
        )

    def trigger_anomaly_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Servicio manual para forzar una falla y evaluar la captura de logs/rosbag."""
        self.anomaly_active = request.data
        if self.anomaly_active:
            self.anomaly_type = "CRITICAL_JOINT_JITTER"
            self.get_logger().warn("⚠️ [INSPECTION] Inyección de anomalía manual ACTIVADA.")
            response.success = True
            response.message = "Anomalía 'CRITICAL_JOINT_JITTER' activada."
        else:
            self.anomaly_type = "NONE"
            self.get_logger().info("✅ [RECOVERY] Inyección de anomalía DESACTIVADA. Sistema normalizado.")
            response.success = True
            response.message = "Sistema restaurado a condición nominal."
        return response

    def dump_flight_recorder_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Vuelca el contenido del ring buffer a los logs como reporte post-mortem."""
        count = len(self.flight_buffer)
        self.get_logger().warn(f"📦 [FLIGHT RECORDER DUMP] Vaciando {count} muestras del buffer de memoria...")

        for i, sample in enumerate(self.flight_buffer):
            # Formatear muestra como DEBUG para no saturar la consola estándar
            self.get_logger().debug(
                f"[T-{count - i}] t={sample['timestamp']:.2f} | "
                f"j1={sample['j1']:.3f}, j4={sample['j4']:.3f} | "
                f"jitter={sample['jitter']:.4f} | status={sample['status']}"
            )

        response.success = True
        response.message = f"Buffer volcado exitosamente ({count} muestras disponibles en DEBUG log)."
        return response

    def telemetry_loop(self):
        """Bucle principal de telemetría y evaluación de salud del sistema."""
        now = self.get_clock().now()
        dt = self.timer_period
        elapsed = time.time() - self.start_time

        # 1. Simular movimiento articular (Kinova Gen3)
        # Movimiento sinusoidal normal
        for i in range(7):
            self.joint_positions[i] = math.sin(elapsed * (0.5 + i * 0.1))

        # Si hay anomalía activa, introducimos ruido no Gaussiano y vibración en joint_4
        jitter_val = 0.002 * (math.sin(elapsed * 10.0))

        if self.anomaly_active:
            # Falla inducida: Salto brusco y vibración en articulación 4
            jitter_val = 0.45 * (math.sin(elapsed * 50.0) + math.cos(elapsed * 25.0))
            self.joint_positions[3] += jitter_val  # joint_4
            status_str = "CRITICAL_FAULT: HIGH_JITTER_DETECTED"
        else:
            status_str = "HEALTH_NOMINAL"

        # 2. Publicación de mensajes ROS 2
        # JointStates
        js_msg = JointState()
        js_msg.header.stamp = now.to_msg()
        js_msg.header.frame_id = "base_link"
        js_msg.name = self.joint_names
        js_msg.position = self.joint_positions
        self.pub_joint_states.publish(js_msg)

        # Jitter métrico
        jitter_msg = Float32()
        jitter_msg.data = float(jitter_val)
        self.pub_jitter.publish(jitter_msg)

        # Estado de salud
        status_msg = String()
        status_msg.data = status_str
        self.pub_status.publish(status_msg)

        # 3. Guardar en Flight Recorder (Circular Buffer)
        if self.enable_buffer:
            self.flight_buffer.append({
                'timestamp': time.time(),
                'j1': self.joint_positions[0],
                'j4': self.joint_positions[3],
                'jitter': jitter_val,
                'status': status_str
            })

        # 4. Estrategia de Logging Multicapa y Throttled
        # DEBUG: Log denso por ciclo
        self.get_logger().debug(
            f"🔄 Telemetry cycle: j1={self.joint_positions[0]:.3f}, j4={self.joint_positions[3]:.3f}, jitter={jitter_val:.4f}"
        )

        # INFO: Resumen periódico cada 2 segundos (Throttling manual)
        curr_t = time.time()
        if curr_t - self.last_log_time >= 2.0:
            self.get_logger().info(
                f"📊 [STATUS] Kinova Arm en {self.ns} | Health: {status_str}"
            )
            self.last_log_time = curr_t

        # WARN / ERROR / FATAL: Eventos excepcionales
        if abs(jitter_val) > 0.3:
            self.get_logger().error(
                f"🚨 [FAULT TRIGGERED] Jitter articular excesivo ({jitter_val:.4f} > 0.3000)! Posible problema con el controlador de trayectoria."
            )
        elif abs(jitter_val) > 0.05:
            self.get_logger().warn(
                f"⚠️ [VIBRATION WARNING] Jitter elevado ({jitter_val:.4f}). Revisar suavidad de metas articulares."
            )


def main(args=None):
    rclpy.init(args=args)
    node = FlightRecorderTelemetryDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Finalización solicitada por el usuario (Ctrl+C).")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
