# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

r"""
Cliente de trayectoria articular segura para el Kinova Gen3 (RF-06).

Envía **una** meta articular previamente aprobada al servidor de acción estándar
``control_msgs/action/FollowJointTrajectory`` que expone ``joint_trajectory_controller``,
después de superar una batería completa de validaciones locales.

Orden de ejecución::

    1. Validar la configuración de seguridad cargada del YAML.
    2. Esperar telemetría fresca y completa de /joint_states.
    3. Construir la meta a partir de la pose aprobada, no de valores en código.
    4. Validar límites, delta máximo, duración y habilitación explícita.
    5. Mostrar el resumen al operador (y pedir confirmación si es interactivo).
    6. Esperar explícitamente al servidor de acción.
    7. Enviar, seguir el feedback y reportar resultado y código de error.

Modo seco (por defecto)::

    ros2 run burger_kinova_reference safe_trajectory_client --ros-args \\
        -p dry_run:=true

En modo seco se ejecuta **toda** la validación y no se contacta al servidor de acción.
Es el modo que debe usarse para documentar la prueba de aceptación PA-07.

Códigos de salida del proceso: ``0`` éxito o validación seca superada, ``1`` meta
bloqueada por seguridad, ``2`` fallo de infraestructura (sin telemetría, sin servidor
de acción), ``3`` meta rechazada o abortada por el controlador.
"""

import sys
import threading
import time
from typing import Dict, List, Optional

from action_msgs.msg import GoalStatus

from builtin_interfaces.msg import Duration

from burger_kinova_reference.link_metrics import LinkHealth, validate_joint_state
from burger_kinova_reference.logging_support import (
    declare_logging_parameters,
    DynamicLogLevel,
    log_logging_banner,
    ThrottledLogger,
)
from burger_kinova_reference.safety import (
    format_goal_report,
    validate_goal,
    validate_safety_config,
)

from control_msgs.action import FollowJointTrajectory

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


EXIT_OK = 0
EXIT_BLOCKED = 1
EXIT_INFRASTRUCTURE = 2
EXIT_REJECTED = 3

#: Traducción de los códigos de error de ``FollowJointTrajectory.Result``.
ERROR_CODE_NAMES = {
    FollowJointTrajectory.Result.SUCCESSFUL: 'SUCCESSFUL',
    FollowJointTrajectory.Result.INVALID_GOAL: 'INVALID_GOAL',
    FollowJointTrajectory.Result.INVALID_JOINTS: 'INVALID_JOINTS',
    FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP: 'OLD_HEADER_TIMESTAMP',
    FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED: 'PATH_TOLERANCE_VIOLATED',
    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED: 'GOAL_TOLERANCE_VIOLATED',
}


class SafeTrajectoryClient(Node):
    """Nodo cliente que valida y envía una única meta articular supervisada."""

    def __init__(self):
        """Declarar parámetros y construir las interfaces del cliente."""
        super().__init__('safe_trajectory_client')

        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('trajectory_action_name',
                               '/joint_trajectory_controller/follow_joint_trajectory')
        self.declare_parameter('use_fake_hardware', True)
        self.declare_parameter('enable_motion', False)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('require_operator_confirmation', True)
        self.declare_parameter(
            'expected_joints',
            ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
        )
        self.declare_parameter('joint_state_timeout_s', 1.0)
        self.declare_parameter('min_joint_state_hz', 20.0)
        self.declare_parameter('max_plausible_joint_rad', 100.0)
        self.declare_parameter('min_rate_observation_s', 0.5)
        self.declare_parameter('max_joint_delta_rad', 0.10)
        self.declare_parameter('trajectory_duration_s', 5.0)
        self.declare_parameter(
            'safe_joint_positions_rad', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter(
            'joint_min_rad', [-3.14, -2.24, -2.57, -3.14, -2.09, -3.14])
        self.declare_parameter(
            'joint_max_rad', [3.14, 2.24, 2.57, 3.14, 2.09, 3.14])
        self.declare_parameter('action_server_timeout_s', 10.0)
        self.declare_parameter('goal_result_timeout_s', 30.0)
        declare_logging_parameters(self)

        self._log = ThrottledLogger(
            self.get_logger(), float(self.get_parameter('log_throttle_period_s').value))
        self._level = DynamicLogLevel(self, self._log)
        log_logging_banner(self)

        self._expected_joints: List[str] = list(self.get_parameter('expected_joints').value)
        self._safe_positions = list(self.get_parameter('safe_joint_positions_rad').value)
        self._joint_min = list(self.get_parameter('joint_min_rad').value)
        self._joint_max = list(self.get_parameter('joint_max_rad').value)
        self._max_delta = float(self.get_parameter('max_joint_delta_rad').value)
        self._max_abs_rad = float(self.get_parameter('max_plausible_joint_rad').value)
        self._duration_s = float(self.get_parameter('trajectory_duration_s').value)
        self._enable_motion = bool(self.get_parameter('enable_motion').value)
        self._use_fake_hardware = bool(self.get_parameter('use_fake_hardware').value)
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._confirm = bool(self.get_parameter('require_operator_confirmation').value)
        self._action_name = str(self.get_parameter('trajectory_action_name').value)
        self._server_timeout = float(self.get_parameter('action_server_timeout_s').value)
        self._result_timeout = float(self.get_parameter('goal_result_timeout_s').value)

        self._health = LinkHealth(
            expected_joints=self._expected_joints,
            timeout_s=float(self.get_parameter('joint_state_timeout_s').value),
            min_hz=float(self.get_parameter('min_joint_state_hz').value),
            min_span_s=float(self.get_parameter('min_rate_observation_s').value),
            max_abs_rad=self._max_abs_rad,
        )
        self._lock = threading.Lock()

        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_state_topic').value),
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self._action_client = ActionClient(self, FollowJointTrajectory, self._action_name)

        modo = 'FAKE' if self._use_fake_hardware else 'HARDWARE REAL'
        self.get_logger().info(
            f'safe_trajectory_client iniciado | acción={self._action_name} '
            f'| modo={modo} | dry_run={self._dry_run} '
            f'| enable_motion={self._enable_motion}'
        )

    # -------------------------------------------------------------- telemetría ---
    def _on_joint_state(self, msg: JointState) -> None:
        """
        Acumular el estado articular más reciente.

        :param msg: mensaje ``/joint_states`` recibido.
        """
        validation = validate_joint_state(
            msg.name, msg.position, self._expected_joints, self._max_abs_rad)
        with self._lock:
            self._health.update(validation, time.monotonic())
        if not validation.valid:
            self._log.warn(f'[TELEMETRÍA] mensaje descartado: {validation.reason}')
        else:
            self._log.debug(f'[TELEMETRÍA] {self._health.hz():.1f} Hz')

    def _snapshot(self):
        """
        Obtener de forma segura la última telemetría válida.

        :returns: tupla ``(posiciones, telemetría_fresca, frecuencia)``.
        """
        with self._lock:
            now = time.monotonic()
            hz = self._health.hz() if self._health.rate_is_reliable() else 0.0
            return self._health.positions, self._health.is_fresh(now), hz

    def _wait_for_telemetry(self, timeout_s: float = 10.0) -> bool:
        """
        Esperar hasta disponer de telemetría fresca y completa.

        :param timeout_s: tiempo máximo de espera.
        :returns: ``True`` si llegó telemetría utilizable.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            positions, fresh, hz = self._snapshot()
            # Se exige además que la frecuencia ya sea creíble: al suscribirse, DDS
            # entrega de golpe los mensajes encolados y una lectura inmediata reportaría
            # miles de Hz en la evidencia de laboratorio.
            if fresh and hz > 0.0 and len(positions) == len(self._expected_joints):
                return True
            time.sleep(0.1)
        return False

    # ------------------------------------------------------------- secuencia ----
    def run(self) -> int:
        """
        Ejecutar la secuencia completa de validación y envío.

        :returns: código de salida del proceso.
        """
        config_errors = validate_safety_config(
            self._expected_joints,
            self._safe_positions,
            self._joint_min,
            self._joint_max,
            self._max_delta,
            self._duration_s,
        )
        if config_errors:
            for error in config_errors:
                self.get_logger().error(f'[CONFIGURACIÓN INSEGURA] {error}')
            self.get_logger().error(
                'No se construye ninguna meta con una configuración de seguridad inválida.')
            return EXIT_BLOCKED

        self.get_logger().info('Esperando telemetría fresca de /joint_states...')
        if not self._wait_for_telemetry():
            self.get_logger().error(
                '[INFRAESTRUCTURA] No llegó telemetría válida. Verifica que el driver esté '
                'corriendo, que ROS_DOMAIN_ID coincida entre estaciones y que la red DDS '
                'permita el descubrimiento (ros2 topic hz /joint_states).'
            )
            return EXIT_INFRASTRUCTURE

        positions, fresh, hz = self._snapshot()
        self.get_logger().info(
            f'Telemetría disponible: {len(positions)} articulaciones a {hz:.1f} Hz.')

        validation = validate_goal(
            expected_joints=self._expected_joints,
            target_positions=self._safe_positions,
            current_positions=positions,
            joint_min=self._joint_min,
            joint_max=self._joint_max,
            max_joint_delta_rad=self._max_delta,
            enable_motion=self._enable_motion,
            use_fake_hardware=self._use_fake_hardware,
            telemetry_fresh=fresh,
            duration_s=self._duration_s,
            min_duration_s=self._duration_s,
        )
        for line in format_goal_report(
            self._expected_joints, self._safe_positions, positions, validation, self._duration_s
        ):
            self.get_logger().info(line)

        if self._dry_run:
            self.get_logger().info(
                '[MODO SECO] dry_run=true: la validación terminó aquí y NO se contactó el '
                'servidor de acción. Repite con -p dry_run:=false para enviar la meta.'
            )
            return EXIT_OK if validation.allowed else EXIT_BLOCKED

        if not validation.allowed:
            self.get_logger().error(
                f'[SEGURIDAD] Meta bloqueada antes de contactar el servidor de acción: '
                f'{validation.reason}'
            )
            return EXIT_BLOCKED

        if not self._action_client.wait_for_server(timeout_sec=self._server_timeout):
            self.get_logger().error(
                f'[INFRAESTRUCTURA] El servidor de acción {self._action_name} no apareció '
                f'en {self._server_timeout:.1f} s. Comprueba que joint_trajectory_controller '
                f'esté activo (ros2 control list_controllers) y que la acción sea visible '
                f'(ros2 action list).'
            )
            return EXIT_INFRASTRUCTURE

        if not self._confirm_with_operator():
            self.get_logger().warn('[SEGURIDAD] Envío cancelado por el operador.')
            return EXIT_BLOCKED

        return self._send_goal(positions)

    def _confirm_with_operator(self) -> bool:
        """
        Solicitar confirmación por teclado cuando la ejecución es interactiva.

        En una ejecución no interactiva (por ejemplo desde un launch o en CI) no hay
        terminal disponible; en ese caso la confirmación se considera implícita en el
        hecho de haber fijado ``enable_motion:=true`` y ``dry_run:=false``.

        :returns: ``True`` si se autoriza el envío.
        """
        if not self._confirm:
            return True
        if not sys.stdin or not sys.stdin.isatty():
            self.get_logger().warn(
                '[SEGURIDAD] Ejecución no interactiva: se omite la confirmación por teclado. '
                'La autorización queda registrada en enable_motion:=true y dry_run:=false.'
            )
            return True
        target = 'HARDWARE REAL' if not self._use_fake_hardware else 'hardware simulado'
        try:
            answer = input(
                f'\n¿Confirmas el envío de la meta al {target}? '
                f'Escribe "si" para continuar: '
            )
        except (EOFError, KeyboardInterrupt):
            return False
        return answer.strip().lower() in ('si', 'sí', 's', 'yes', 'y')

    def _build_goal(self, current: Dict[str, float]) -> FollowJointTrajectory.Goal:
        """
        Construir la meta a partir de la pose aprobada y el estado actual.

        :param current: posiciones actuales por articulación.
        :returns: la meta lista para enviarse.
        """
        del current  # el estado actual ya se usó en la validación; la meta es absoluta
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self._expected_joints)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in self._safe_positions]
        point.velocities = [0.0] * len(self._expected_joints)
        seconds = int(self._duration_s)
        point.time_from_start = Duration(
            sec=seconds, nanosec=int((self._duration_s - seconds) * 1e9))
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        return goal

    def _send_goal(self, current: Dict[str, float]) -> int:
        """
        Enviar la meta y seguir su ciclo de vida hasta el resultado.

        :param current: posiciones actuales conocidas.
        :returns: código de salida del proceso.
        """
        goal = self._build_goal(current)
        self.get_logger().warn(
            f'[ENVÍO] Enviando meta de {len(goal.trajectory.points)} punto(s) a '
            f'{self._action_name} con duración {self._duration_s:.2f} s.'
        )
        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        if not self._await(send_future, self._server_timeout):
            self.get_logger().error(
                '[INFRAESTRUCTURA] El servidor no respondió al envío de la meta dentro del '
                'tiempo previsto.')
            return EXIT_INFRASTRUCTURE

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                '[RESULTADO] Meta RECHAZADA por el controlador. Revisa que los nombres de '
                'articulación coincidan con los del controlador y que este esté activo.')
            return EXIT_REJECTED
        self.get_logger().info('[RESULTADO] Meta ACEPTADA por el controlador.')

        result_future = goal_handle.get_result_async()
        if not self._await(result_future, self._result_timeout):
            self.get_logger().error(
                f'[TIMEOUT] Sin resultado tras {self._result_timeout:.1f} s. Se solicita la '
                f'cancelación ordenada de la meta.')
            cancel_future = goal_handle.cancel_goal_async()
            self._await(cancel_future, 5.0)
            self.get_logger().warn('[TIMEOUT] Cancelación solicitada; el nodo termina limpio.')
            return EXIT_REJECTED

        wrapped = result_future.result()
        status = wrapped.status
        result = wrapped.result
        code_name = ERROR_CODE_NAMES.get(result.error_code, f'DESCONOCIDO({result.error_code})')

        if status == GoalStatus.STATUS_SUCCEEDED and \
                result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(
                f'[RESULTADO] Trayectoria completada con éxito | error_code={code_name}')
            return EXIT_OK

        status_names = {
            GoalStatus.STATUS_ABORTED: 'ABORTADA por el controlador',
            GoalStatus.STATUS_CANCELED: 'CANCELADA',
            GoalStatus.STATUS_SUCCEEDED: 'terminada con advertencias',
        }
        self.get_logger().error(
            f'[RESULTADO] Meta {status_names.get(status, f"en estado {status}")} | '
            f'error_code={code_name} | error_string="{result.error_string}"'
        )
        return EXIT_REJECTED

    def _on_feedback(self, feedback_msg) -> None:
        """
        Registrar el avance de la trayectoria con limitación de frecuencia.

        :param feedback_msg: mensaje de feedback del servidor de acción.
        """
        feedback = feedback_msg.feedback
        errors = list(feedback.error.positions) if feedback.error.positions else []
        peak = max((abs(value) for value in errors), default=0.0)
        self._log.info(
            f'[FEEDBACK] t={feedback.actual.time_from_start.sec}.'
            f'{feedback.actual.time_from_start.nanosec // 1000000:03d} s | '
            f'error máximo por articulación={peak:.5f} rad',
            throttle=True,
        )

    @staticmethod
    def _await(future, timeout_s: float) -> bool:
        """
        Esperar a que un futuro se complete, con el ejecutor girando en otro hilo.

        :param future: futuro a esperar.
        :param timeout_s: tiempo máximo de espera.
        :returns: ``True`` si el futuro se completó a tiempo.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.05)
        return future.done()


def main(args: Optional[List[str]] = None) -> None:
    """
    Punto de entrada del ejecutable ``safe_trajectory_client``.

    El ejecutor gira en un hilo dedicado para que la secuencia principal pueda esperar
    futuros y pedir confirmación por teclado sin bloquear los callbacks de telemetría.

    :param args: argumentos de línea de comandos.
    """
    rclpy.init(args=args)
    node: Optional[SafeTrajectoryClient] = None
    executor = SingleThreadedExecutor()
    exit_code = EXIT_INFRASTRUCTURE
    spin_thread: Optional[threading.Thread] = None
    try:
        node = SafeTrajectoryClient()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        exit_code = node.run()
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warn('Interrupción por teclado: no se envió ninguna meta nueva.')
        exit_code = EXIT_BLOCKED
    except Exception as exc:  # noqa: BLE001 - se registra la causa antes de terminar
        logger = (node.get_logger() if node is not None
                  else rclpy.logging.get_logger('safe_trajectory_client'))
        logger.fatal(f'Terminación por excepción no controlada: {exc}')
        exit_code = EXIT_INFRASTRUCTURE
    finally:
        # Cierre ordenado y a prueba de una segunda señal: primero se detiene el ejecutor,
        # luego se libera el nodo. Ningún fallo de teardown debe enmascarar el código de
        # salida real de la validación.
        try:
            executor.shutdown()
            if spin_thread is not None:
                spin_thread.join(timeout=2.0)
        except BaseException:  # noqa: B036 - cierre best-effort
            pass
        try:
            if node is not None:
                node.destroy_node()
        except BaseException:  # noqa: B036 - cierre best-effort
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except BaseException:  # noqa: B036 - cierre best-effort
            pass
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
