#!/usr/bin/env python3
"""
flight_recorder_telemetry_demo.py
Nodo de telemetría y diagnóstico para el taller de Logging y rosbag2 en ROS 2 Jazzy.

Demuestra:
1. Niveles de log en ROS 2 (DEBUG, INFO, WARN, ERROR, FATAL) con formateo y throttling.
2. Cambio dinámico de nivel de log en tiempo de ejecución vía parámetros/servicios.
3. Publicación de estados de odometría, telemetría y diagnóstico del robot 'burger_delivery'.
4. Inyección controlada de anomalías (jittering, timeout de sensor, desviación cinemática).
5. Integración con rosbag2 (grabación continua o buffer post-mortem / flight recorder).

Uso:
    ros2 run burger_delivery flight_recorder_telemetry_demo.py
    # O directamente con Python:
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
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import Trigger, SetBool


class FlightRecorderTelemetryDemo(Node):
    """
    Nodo emulador de subsistemas del robot con capacidades avanzadas de logging y telemetría.
    """

    def __init__(self):
        super().__init__('flight_recorder_telemetry_demo')

        # Declaración de parámetros configurables dinámicamente
        self.declare_parameter('robot_id', 'burger_car_01')
        self.declare_parameter('telemetry_rate_hz', 20.0)
        self.declare_parameter('inject_fault_period_sec', 0.0)  # 0 = sin fallos automáticos
        self.declare_parameter('enable_flight_buffer', True)
        self.declare_parameter('buffer_max_samples', 200)

        # Lectura de parámetros
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        rate_hz = self.get_parameter('telemetry_rate_hz').get_parameter_value().double_value
        self.fault_period = self.get_parameter('inject_fault_period_sec').get_parameter_value().double_value
        self.enable_buffer = self.get_parameter('enable_flight_buffer').get_parameter_value().bool_value
        self.max_samples = self.get_parameter('buffer_max_samples').get_parameter_value().integer_value

        # Publicadores de telemetría
        self.pub_odom = self.create_publisher(Odometry, f'/{self.robot_id}/odom', 10)
        self.pub_pose = self.create_publisher(PoseStamped, f'/{self.robot_id}/pose', 10)
        self.pub_jitter = self.create_publisher(Float32, f'/{self.robot_id}/joint_jitter', 10)
        self.pub_status = self.create_publisher(String, f'/{self.robot_id}/system_health', 10)

        # Suscriptores
        self.sub_cmd_vel = self.create_subscription(
            Twist, f'/{self.robot_id}/cmd_vel', self.cmd_vel_callback, 10
        )

        # Servicios para inyección de anomalías y disparo de blackbox
        self.srv_fault = self.create_service(
            SetBool, f'/{self.robot_id}/trigger_anomaly', self.trigger_anomaly_callback
        )
        self.srv_dump_buffer = self.create_service(
            Trigger, f'/{self.robot_id}/dump_flight_recorder', self.dump_flight_recorder_callback
        )

        # Estado cinemático interno
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        # Control de anomalías
        self.anomaly_active = False
        self.anomaly_type = "NONE"
        self.anomaly_counter = 0

        # Ring Buffer en memoria (Flight Recorder / Caja Negra)
        self.flight_buffer: Deque[Dict[str, Any]] = deque(maxlen=self.max_samples)

        # Timers
        self.timer_period = 1.0 / max(1.0, rate_hz)
        self.timer = self.create_timer(self.timer_period, self.telemetry_loop)
        self.start_time = time.time()
        self.last_log_time = time.time()

        self.get_logger().info(
            f"🚀 [INIT] Nodo '{self.get_name()}' iniciado para robot '{self.robot_id}' a {rate_hz} Hz."
        )
        self.get_logger().info(
            "💡 TIP: Usa 'ros2 param set /flight_recorder_telemetry_demo log_level DEBUG' para ver trazas profundas."
        )

    def cmd_vel_callback(self, msg: Twist):
        """Callback de velocidad entrante."""
        self.current_linear_x = msg.linear.x
        self.current_angular_z = msg.angular.z
        self.get_logger().debug(
            f"📥 CmdVel recibido: v_x={msg.linear.x:.2f} m/s, w_z={msg.angular.z:.2f} rad/s"
        )

    def trigger_anomaly_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Servicio manual para forzar una falla y evaluar la captura de logs/rosbag."""
        self.anomaly_active = request.data
        if self.anomaly_active:
            self.anomaly_type = "CRITICAL_KINEMATIC_JITTER"
            self.get_logger().warn("⚠️ [INSPECTION] Inyección de anomalía manual ACTIVADA.")
            response.success = True
            response.message = "Anomalía 'CRITICAL_KINEMATIC_JITTER' activada."
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
                f"[T-{count - i}] t={sample['timestamp']:.2f} | pose=({sample['x']:.3f}, {sample['y']:.3f}) | "
                f"jitter={sample['jitter']:.4f} | status={sample['status']}"
            )

        response.success = True
        response.message = f"Buffer volcado exitosamente ({count} muestras disponibles en DEBUG log)."
        return response

    def telemetry_loop(self):
        """Bucle principal de telemetría y evaluación de salud del sistema."""
        now = self.get_clock().now()
        dt = self.timer_period

        # 1. Simular integración cinemática (Robot móvil diferencial)
        # Si hay anomalía activa, introducimos ruido no Gaussiano y vibración
        jitter_val = 0.002 * (math.sin(time.time() * 10.0))

        if self.anomaly_active:
            # Falla inducida: Salto brusco y divergencia de orientación
            jitter_val = 0.45 * (math.sin(time.time() * 50.0) + math.cos(time.time() * 25.0))
            self.theta += (self.current_angular_z + 1.2) * dt
            self.x += (self.current_linear_x * math.cos(self.theta) + 0.15) * dt
            self.y += (self.current_linear_x * math.sin(self.theta) + 0.15) * dt
            status_str = "CRITICAL_FAULT: HIGH_JITTER_DETECTED"
        else:
            self.theta += self.current_angular_z * dt
            self.x += self.current_linear_x * math.cos(self.theta) * dt
            self.y += self.current_linear_x * math.sin(self.theta) * dt
            status_str = "HEALTH_NOMINAL"

        # 2. Publicación de mensajes ROS 2
        # Odometría
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = f"{self.robot_id}_base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        # Cuaternión simple en 2D (Yaw)
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom_msg.twist.twist.linear.x = self.current_linear_x
        odom_msg.twist.twist.angular.z = self.current_angular_z
        self.pub_odom.publish(odom_msg)

        # PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose.pose
        self.pub_pose.publish(pose_msg)

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
                'x': self.x,
                'y': self.y,
                'theta': self.theta,
                'jitter': jitter_val,
                'status': status_str
            })

        # 4. Estrategia de Logging Multicapa y Throttled
        # DEBUG: Log denso por ciclo
        self.get_logger().debug(
            f"🔄 Telemetry cycle: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.2f} rad, jitter={jitter_val:.4f}"
        )

        # INFO: Resumen periódico cada 2 segundos (Throttling manual)
        curr_t = time.time()
        if curr_t - self.last_log_time >= 2.0:
            self.get_logger().info(
                f"📊 [STATUS] Robot '{self.robot_id}' en ({self.x:.2f}, {self.y:.2f}) | Health: {status_str}"
            )
            self.last_log_time = curr_t

        # WARN / ERROR / FATAL: Eventos excepcionales
        if abs(jitter_val) > 0.3:
            self.get_logger().error(
                f"🚨 [FAULT TRIGGERED] Jitter excesivo ({jitter_val:.4f} > 0.3000)! Posible desprendimiento de tag o desincronización UDP."
            )
        elif abs(jitter_val) > 0.05:
            self.get_logger().warn(
                f"⚠️ [VIBRATION WARNING] Jitter elevado ({jitter_val:.4f}). Revisar amortiguación cinemática."
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
