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
Secuencia articular autónoma y supervisada para el Kinova Gen3.

Extiende :mod:`safe_trajectory_client`, que envía **una** meta, a una serie de puntos
ejecutados en orden sin intervención. La diferencia importante no es el bucle: es que
**antes de cada punto se vuelve a leer la pose real y se revalida todo**.

Por qué eso importa. Si se planificara la serie completa por adelantado a partir de la
pose inicial, el segundo punto se validaría contra dónde el robot *debería* estar, no
contra dónde está. Cualquier desviación —una tolerancia de seguimiento, un tramo
abortado, un empujón— se arrastraría en silencio y el desplazamiento real podría superar
``max_joint_delta_rad`` sin que ninguna comprobación lo advirtiera. Releyendo antes de
cada envío, cada tramo se acota contra el estado verdadero del brazo.

Los puntos se declaran como **deltas relativos** a la pose capturada al arrancar, no como
posiciones absolutas. Así la secuencia es ejecutable desde donde quiera que esté el brazo,
sin volver a aprobar una pose fija cada vez, y el recorrido total queda limitado por
``max_sequence_excursion_rad``.

La secuencia se detiene en cuanto un punto no supera la validación, el enlace se degrada o
el controlador rechaza o aborta una meta. No reintenta: en este proyecto no hay
recuperación automática de fallas.

Ejemplo de uso (modo seco, no toca el robot)::

    ros2 run burger_kinova_reference safe_sequence_client --ros-args \
        --params-file <config> -p dry_run:=true
"""

import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

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

ERROR_CODE_NAMES = {
    FollowJointTrajectory.Result.SUCCESSFUL: 'SUCCESSFUL',
    FollowJointTrajectory.Result.INVALID_GOAL: 'INVALID_GOAL',
    FollowJointTrajectory.Result.INVALID_JOINTS: 'INVALID_JOINTS',
    FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP: 'OLD_HEADER_TIMESTAMP',
    FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED: 'PATH_TOLERANCE_VIOLATED',
    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED: 'GOAL_TOLERANCE_VIOLATED',
}


def descomponer_secuencia(plano: List[float], n_articulaciones: int
                          ) -> List[List[float]]:
    """
    Partir la lista plana de deltas del YAML en puntos de ``n_articulaciones``.

    ROS 2 no admite arreglos anidados como parámetro, así que la secuencia viaja
    aplanada: los primeros ``n`` valores son el punto 1, los siguientes ``n`` el punto 2,
    y así sucesivamente.

    :param plano: lista aplanada de deltas, en radianes.
    :param n_articulaciones: número de articulaciones por punto.
    :returns: lista de puntos, cada uno con ``n_articulaciones`` deltas.
    :raises ValueError: si la longitud no es múltiplo del número de articulaciones.
    """
    if n_articulaciones <= 0:
        raise ValueError('el número de articulaciones debe ser positivo')
    if not plano:
        return []
    if len(plano) % n_articulaciones != 0:
        raise ValueError(
            f'la secuencia tiene {len(plano)} valores, que no es múltiplo de '
            f'{n_articulaciones}: cada punto debe declarar un delta por articulación'
        )
    return [list(plano[i:i + n_articulaciones])
            for i in range(0, len(plano), n_articulaciones)]


def validar_excursion(puntos: List[List[float]], maxima: float) -> List[str]:
    """
    Comprobar que ningún punto se aleja del origen más de lo permitido.

    ``max_joint_delta_rad`` acota cada tramo por separado; sin este límite adicional una
    sucesión de tramos pequeños podría llevar el brazo arbitrariamente lejos de la pose
    de arranque.

    :param puntos: deltas acumulados respecto al origen, por punto.
    :param maxima: excursión máxima admitida, en radianes.
    :returns: lista de errores; vacía si todos los puntos están dentro.
    """
    errores: List[str] = []
    for indice, punto in enumerate(puntos, start=1):
        for articulacion, delta in enumerate(punto, start=1):
            if abs(delta) > maxima:
                errores.append(
                    f'punto {indice}, joint_{articulacion}: la excursión '
                    f'{abs(delta):.4f} rad respecto al origen supera '
                    f'max_sequence_excursion_rad={maxima:.4f} rad'
                )
    return errores


class SafeSequenceClient(Node):
    """Ejecuta una serie de metas articulares revalidando el estado antes de cada una."""

    def __init__(self):
        """Declarar parámetros y construir las interfaces."""
        super().__init__('safe_sequence_client')

        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('trajectory_action_name',
                               '/joint_trajectory_controller/follow_joint_trajectory')
        self.declare_parameter('use_fake_hardware', True)
        self.declare_parameter('enable_motion', False)
        self.declare_parameter('dry_run', True)
        self.declare_parameter(
            'expected_joints',
            ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        self.declare_parameter('joint_state_timeout_s', 1.0)
        self.declare_parameter('min_joint_state_hz', 20.0)
        self.declare_parameter('max_plausible_joint_rad', 100.0)
        self.declare_parameter('min_rate_observation_s', 0.5)
        self.declare_parameter('max_joint_delta_rad', 0.10)
        self.declare_parameter('trajectory_duration_s', 5.0)
        self.declare_parameter(
            'joint_min_rad', [-3.14, -2.24, -2.57, -3.14, -2.09, -3.14])
        self.declare_parameter(
            'joint_max_rad', [3.14, 2.24, 2.57, 3.14, 2.09, 3.14])
        self.declare_parameter('action_server_timeout_s', 10.0)
        self.declare_parameter('goal_result_timeout_s', 30.0)
        # ---- propios de la secuencia ----------------------------------------------
        self.declare_parameter('sequence_deltas_rad', [0.0])
        self.declare_parameter('sequence_point_duration_s', 4.0)
        self.declare_parameter('sequence_pause_s', 1.0)
        self.declare_parameter('max_sequence_excursion_rad', 0.60)
        self.declare_parameter('return_to_origin', True)
        declare_logging_parameters(self)

        self._log = ThrottledLogger(
            self.get_logger(), float(self.get_parameter('log_throttle_period_s').value))
        self._level = DynamicLogLevel(self, self._log)
        log_logging_banner(self)

        self._expected_joints: List[str] = list(
            self.get_parameter('expected_joints').value)
        self._joint_min = list(self.get_parameter('joint_min_rad').value)
        self._joint_max = list(self.get_parameter('joint_max_rad').value)
        self._max_delta = float(self.get_parameter('max_joint_delta_rad').value)
        self._max_abs_rad = float(self.get_parameter('max_plausible_joint_rad').value)
        self._enable_motion = bool(self.get_parameter('enable_motion').value)
        self._use_fake_hardware = bool(self.get_parameter('use_fake_hardware').value)
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._action_name = str(self.get_parameter('trajectory_action_name').value)
        self._server_timeout = float(self.get_parameter('action_server_timeout_s').value)
        self._result_timeout = float(self.get_parameter('goal_result_timeout_s').value)
        self._punto_duracion = float(
            self.get_parameter('sequence_point_duration_s').value)
        self._pausa = float(self.get_parameter('sequence_pause_s').value)
        self._excursion_maxima = float(
            self.get_parameter('max_sequence_excursion_rad').value)
        self._volver_al_origen = bool(self.get_parameter('return_to_origin').value)

        self._health = LinkHealth(
            expected_joints=self._expected_joints,
            timeout_s=float(self.get_parameter('joint_state_timeout_s').value),
            min_hz=float(self.get_parameter('min_joint_state_hz').value),
            min_span_s=float(self.get_parameter('min_rate_observation_s').value),
            max_abs_rad=self._max_abs_rad,
        )
        self._lock = threading.Lock()
        self._detener = threading.Event()

        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_state_topic').value),
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self._action_client = ActionClient(self, FollowJointTrajectory, self._action_name)

        modo = 'FAKE' if self._use_fake_hardware else 'HARDWARE REAL'
        self.get_logger().info(
            f'safe_sequence_client iniciado | acción={self._action_name} | modo={modo} '
            f'| dry_run={self._dry_run} | enable_motion={self._enable_motion}'
        )

    # ------------------------------------------------------------- telemetría ----
    def _on_joint_state(self, msg: JointState) -> None:
        """Acumular el estado articular más reciente."""
        validacion = validate_joint_state(
            msg.name, msg.position, self._expected_joints, self._max_abs_rad)
        with self._lock:
            self._health.update(validacion, time.monotonic())
        if not validacion.valid:
            self._log.warn(f'[TELEMETRÍA] mensaje descartado: {validacion.reason}')

    def _snapshot(self) -> Tuple[Dict[str, float], bool, float]:
        """Obtener de forma segura la última telemetría válida."""
        with self._lock:
            ahora = time.monotonic()
            hz = self._health.hz() if self._health.rate_is_reliable() else 0.0
            return self._health.positions, self._health.is_fresh(ahora), hz

    def _esperar_telemetria(self, timeout_s: float = 10.0) -> bool:
        """Esperar hasta disponer de telemetría fresca, completa y creíble."""
        limite = time.monotonic() + timeout_s
        while time.monotonic() < limite and not self._detener.is_set():
            posiciones, fresca, hz = self._snapshot()
            if fresca and hz > 0.0 and len(posiciones) == len(self._expected_joints):
                return True
            time.sleep(0.1)
        return False

    # --------------------------------------------------------------- secuencia ---
    def run(self) -> int:
        """
        Ejecutar la secuencia completa.

        :returns: código de salida del proceso.
        """
        errores = validate_safety_config(
            self._expected_joints,
            [0.0] * len(self._expected_joints),
            self._joint_min, self._joint_max,
            self._max_delta, self._punto_duracion,
        )
        if errores:
            for error in errores:
                self.get_logger().error(f'[CONFIGURACIÓN INSEGURA] {error}')
            return EXIT_BLOCKED

        try:
            puntos = descomponer_secuencia(
                list(self.get_parameter('sequence_deltas_rad').value),
                len(self._expected_joints))
        except ValueError as exc:
            self.get_logger().error(f'[SECUENCIA INVÁLIDA] {exc}')
            return EXIT_BLOCKED

        if not puntos or all(all(d == 0.0 for d in p) for p in puntos):
            self.get_logger().error(
                '[SECUENCIA INVÁLIDA] sequence_deltas_rad está vacío o es todo ceros: '
                'no hay ningún movimiento que ejecutar.')
            return EXIT_BLOCKED

        errores = validar_excursion(puntos, self._excursion_maxima)
        if errores:
            for error in errores:
                self.get_logger().error(f'[SECUENCIA INSEGURA] {error}')
            return EXIT_BLOCKED

        if self._volver_al_origen:
            puntos = puntos + [[0.0] * len(self._expected_joints)]

        self.get_logger().info('Esperando telemetría fresca de /joint_states...')
        if not self._esperar_telemetria():
            self.get_logger().error(
                '[INFRAESTRUCTURA] No llegó telemetría válida. Verifica el driver, el '
                'ROS_DOMAIN_ID y la red DDS.')
            return EXIT_INFRASTRUCTURE

        origen, _, hz = self._snapshot()
        self.get_logger().info(
            f'Pose de origen capturada a {hz:.1f} Hz: '
            + ' '.join(f'{j}={origen[j]:+.4f}' for j in self._expected_joints))
        self.get_logger().info(
            f'Secuencia de {len(puntos)} punto(s), {self._punto_duracion:.1f} s por tramo'
            + (' (el último regresa al origen)' if self._volver_al_origen else ''))

        if not self._dry_run:
            if not self._action_client.wait_for_server(timeout_sec=self._server_timeout):
                self.get_logger().error(
                    f'[INFRAESTRUCTURA] El servidor de acción {self._action_name} no '
                    f'apareció en {self._server_timeout:.1f} s.')
                return EXIT_INFRASTRUCTURE

        for indice, deltas in enumerate(puntos, start=1):
            if self._detener.is_set():
                self.get_logger().warn(
                    f'[SECUENCIA] Interrumpida por el operador antes del punto {indice}.')
                return EXIT_BLOCKED
            codigo = self._ejecutar_punto(indice, len(puntos), origen, deltas)
            if codigo != EXIT_OK:
                self.get_logger().error(
                    f'[SECUENCIA] Detenida en el punto {indice} de {len(puntos)}. '
                    f'No se reintenta: este proyecto no recupera fallas automáticamente.')
                return codigo
            if indice < len(puntos) and self._pausa > 0.0:
                time.sleep(self._pausa)

        self.get_logger().info(
            f'[SECUENCIA] Completada: {len(puntos)} punto(s) ejecutados con éxito.')
        return EXIT_OK

    def _ejecutar_punto(self, indice: int, total: int,
                        origen: Dict[str, float], deltas: List[float]) -> int:
        """
        Validar y ejecutar un punto de la secuencia.

        :param indice: número de punto, empezando en 1.
        :param total: total de puntos de la secuencia.
        :param origen: pose capturada al arrancar la secuencia.
        :param deltas: desplazamiento de este punto respecto al origen.
        :returns: código de salida parcial; ``EXIT_OK`` si el punto se completó.
        """
        self.get_logger().info(f'── Punto {indice}/{total} ──')

        # Se relee la pose ANTES de cada punto: validar contra dónde el robot debería
        # estar, y no contra dónde está, dejaría pasar desviaciones acumuladas.
        actual, fresca, _ = self._snapshot()
        meta = [origen[j] + deltas[i]
                for i, j in enumerate(self._expected_joints)]

        validacion = validate_goal(
            expected_joints=self._expected_joints,
            target_positions=meta,
            current_positions=actual,
            joint_min=self._joint_min,
            joint_max=self._joint_max,
            max_joint_delta_rad=self._max_delta,
            enable_motion=self._enable_motion,
            use_fake_hardware=self._use_fake_hardware,
            telemetry_fresh=fresca,
            duration_s=self._punto_duracion,
            min_duration_s=self._punto_duracion,
        )
        for linea in format_goal_report(
                self._expected_joints, meta, actual, validacion, self._punto_duracion):
            self.get_logger().info(linea)

        if self._dry_run:
            self.get_logger().info(
                f'[MODO SECO] Punto {indice}/{total} validado; no se contacta el servidor '
                f'de acción.')
            return EXIT_OK if validacion.allowed else EXIT_BLOCKED

        if not validacion.allowed:
            self.get_logger().error(
                f'[SEGURIDAD] Punto {indice} bloqueado: {validacion.reason}')
            return EXIT_BLOCKED

        return self._enviar(indice, total, meta)

    def _enviar(self, indice: int, total: int, meta: List[float]) -> int:
        """Enviar una meta y seguirla hasta el resultado."""
        trayectoria = JointTrajectory()
        trayectoria.joint_names = list(self._expected_joints)
        punto = JointTrajectoryPoint()
        punto.positions = [float(v) for v in meta]
        punto.velocities = [0.0] * len(self._expected_joints)
        segundos = int(self._punto_duracion)
        punto.time_from_start = Duration(
            sec=segundos, nanosec=int((self._punto_duracion - segundos) * 1e9))
        trayectoria.points = [punto]
        objetivo = FollowJointTrajectory.Goal()
        objetivo.trajectory = trayectoria

        self.get_logger().warn(
            f'[ENVÍO] Punto {indice}/{total} → {self._action_name} '
            f'({self._punto_duracion:.1f} s)')
        futuro = self._action_client.send_goal_async(
            objetivo, feedback_callback=self._on_feedback)
        if not self._await(futuro, self._server_timeout):
            self.get_logger().error('[INFRAESTRUCTURA] El servidor no respondió al envío.')
            return EXIT_INFRASTRUCTURE

        handle = futuro.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'[RESULTADO] Punto {indice} RECHAZADO.')
            return EXIT_REJECTED
        self.get_logger().info(f'[RESULTADO] Punto {indice} ACEPTADO.')

        futuro_resultado = handle.get_result_async()
        if not self._await(futuro_resultado, self._result_timeout):
            self.get_logger().error(
                f'[TIMEOUT] Sin resultado tras {self._result_timeout:.1f} s; se cancela.')
            self._await(handle.cancel_goal_async(), 5.0)
            return EXIT_REJECTED

        envuelto = futuro_resultado.result()
        resultado = envuelto.result
        nombre = ERROR_CODE_NAMES.get(
            resultado.error_code, f'DESCONOCIDO({resultado.error_code})')
        if (envuelto.status == GoalStatus.STATUS_SUCCEEDED
                and resultado.error_code == FollowJointTrajectory.Result.SUCCESSFUL):
            self.get_logger().info(
                f'[RESULTADO] Punto {indice}/{total} completado | error_code={nombre}')
            return EXIT_OK

        self.get_logger().error(
            f'[RESULTADO] Punto {indice} terminó en estado {envuelto.status} | '
            f'error_code={nombre} | error_string="{resultado.error_string}"')
        return EXIT_REJECTED

    def _on_feedback(self, feedback_msg) -> None:
        """Registrar el avance con limitación de frecuencia."""
        feedback = feedback_msg.feedback
        errores = list(feedback.error.positions) if feedback.error.positions else []
        pico = max((abs(v) for v in errores), default=0.0)
        self._log.info(
            f'[FEEDBACK] error máximo por articulación={pico:.5f} rad', throttle=True)

    def solicitar_parada(self) -> None:
        """Pedir que la secuencia se detenga antes del siguiente punto."""
        self._detener.set()

    @staticmethod
    def _await(futuro, timeout_s: float) -> bool:
        """Esperar a que un futuro se complete, con el ejecutor girando aparte."""
        limite = time.monotonic() + timeout_s
        while time.monotonic() < limite:
            if futuro.done():
                return True
            time.sleep(0.05)
        return futuro.done()


def main(args: Optional[List[str]] = None) -> None:
    """Punto de entrada del ejecutable ``safe_sequence_client``."""
    rclpy.init(args=args)
    nodo: Optional[SafeSequenceClient] = None
    ejecutor = SingleThreadedExecutor()
    codigo = EXIT_INFRASTRUCTURE
    hilo: Optional[threading.Thread] = None
    try:
        nodo = SafeSequenceClient()
        ejecutor.add_node(nodo)
        hilo = threading.Thread(target=ejecutor.spin, daemon=True)
        hilo.start()
        codigo = nodo.run()
    except KeyboardInterrupt:
        if nodo is not None:
            nodo.solicitar_parada()
            nodo.get_logger().warn(
                'Interrupción por teclado: no se envía ningún punto más. La meta en curso '
                'la termina el controlador.')
        codigo = EXIT_BLOCKED
    except Exception as exc:  # noqa: BLE001 - se registra la causa antes de terminar
        registro = (nodo.get_logger() if nodo is not None
                    else rclpy.logging.get_logger('safe_sequence_client'))
        registro.fatal(f'Terminación por excepción no controlada: {exc}')
        codigo = EXIT_INFRASTRUCTURE
    finally:
        try:
            ejecutor.shutdown()
            if hilo is not None:
                hilo.join(timeout=2.0)
        except BaseException:  # noqa: B036 - cierre best-effort
            pass
        try:
            if nodo is not None:
                nodo.destroy_node()
        except BaseException:  # noqa: B036 - cierre best-effort
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except BaseException:  # noqa: B036 - cierre best-effort
            pass
    sys.exit(codigo)


if __name__ == '__main__':
    main()
