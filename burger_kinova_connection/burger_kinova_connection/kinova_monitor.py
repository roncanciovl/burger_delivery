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
Monitor del enlace ROS 2 con el manipulador Kinova Gen3 de 7 GDL.

Responsabilidades (RF-03, RF-04, RF-05, RF-08):

* Suscribirse a ``/joint_states`` y validar cada mensaje sin depender del orden del
  arreglo de nombres.
* Medir frecuencia, edad del último mensaje y número de interrupciones.
* Consultar ``/controller_manager/list_controllers`` de forma asíncrona, sin bloquear
  el ejecutor ni terminar el nodo cuando el servicio no está disponible.
* Publicar ``/burger/kinova/diagnostics`` (``diagnostic_msgs/msg/DiagnosticArray``) con
  el estado consolidado del enlace.
* Mantener una caja negra en RAM con la telemetría reciente, volcable bajo demanda.

Decisiones de QoS
-----------------
``/joint_states`` se suscribe con el perfil de **datos de sensor**
(``BEST_EFFORT``, ``KEEP_LAST`` con profundidad 5). Es el perfil correcto para un flujo
periódico de alta frecuencia: ante congestión de red preferimos perder una muestra
antigua a acumular retransmisiones que envejezcan la telemetría. Un suscriptor
``BEST_EFFORT`` es compatible con el publicador ``RELIABLE`` del driver de Kinova, de
modo que la elección no impide el descubrimiento.

``/burger/kinova/diagnostics`` se publica con **comunicación fiable**
(``RELIABLE``, ``VOLATILE``, profundidad 10): un diagnóstico es un evento, no una
muestra; perder la transición a ``ERROR`` invalidaría la evidencia del incidente.

Ejemplo de uso::

    ros2 run burger_kinova_connection kinova_monitor --ros-args \\
        --params-file install/burger_kinova_connection/share/\\
burger_kinova_connection/config/kinova_connection.yaml
"""

import time
from typing import Dict, List, Optional, Tuple

from burger_kinova_connection.flight_recorder import (
    attach_flight_recorder_services,
    FlightRecorder,
)
from burger_kinova_connection.link_metrics import (
    LinkHealth,
    STATE_ERROR,
    STATE_OK,
    STATE_WARN,
    validate_joint_state,
    worst_state,
)
from burger_kinova_connection.logging_support import (
    declare_logging_parameters,
    DynamicLogLevel,
    log_logging_banner,
    StateTransitionLogger,
    ThrottledLogger,
)
from burger_kinova_connection.safety import validate_safety_config
from burger_kinova_connection.station_announcer import (
    construir_anuncio,
    PUERTO_ANUNCIO,
    StationAnnouncer,
)
from burger_kinova_connection.station_identity import describir_estacion

from controller_manager_msgs.srv import ListControllers

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from rcl_interfaces.msg import SetParametersResult

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import JointState

from std_srvs.srv import Trigger


#: Traducción del estado interno al nivel de ``diagnostic_msgs``.
_DIAGNOSTIC_LEVEL = {
    STATE_OK: DiagnosticStatus.OK,
    STATE_WARN: DiagnosticStatus.WARN,
    STATE_ERROR: DiagnosticStatus.ERROR,
}


class KinovaMonitor(Node):
    """Nodo monitor del enlace de telemetría y control con el Kinova Gen3."""

    def __init__(self):
        """Declarar parámetros, construir interfaces y arrancar los temporizadores."""
        super().__init__('kinova_monitor')

        # ---------------------------------------------------------- parámetros ----
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('diagnostics_topic', '/burger/kinova/diagnostics')
        self.declare_parameter('controller_manager_ns', '/controller_manager')
        self.declare_parameter('trajectory_action_name',
                               '/joint_trajectory_controller/follow_joint_trajectory')
        self.declare_parameter('use_fake_hardware', True)
        self.declare_parameter('enable_motion', False)
        # robot_ip y start_driver llegan desde el launch. El monitor NO los usa para
        # conectarse (eso es del driver); los usa para saber si ESTA máquina es la
        # estación anfitriona, y anunciarlo en el diagnóstico.
        self.declare_parameter('robot_ip', '0.0.0.0')
        self.declare_parameter('start_driver', False)
        # Anuncio del rol de la estación para el monitor de red del laboratorio.
        self.declare_parameter('announce_station', True)
        self.declare_parameter('announce_port', PUERTO_ANUNCIO)
        self.declare_parameter('announce_period_s', 5.0)
        self.declare_parameter(
            'expected_joints',
            ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'],
        )
        self.declare_parameter('joint_state_timeout_s', 1.0)
        self.declare_parameter('min_joint_state_hz', 20.0)
        self.declare_parameter('diagnostic_rate_hz', 1.0)
        self.declare_parameter('hz_window_samples', 50)
        self.declare_parameter('min_rate_observation_s', 0.5)
        self.declare_parameter('controller_query_period_s', 2.0)
        self.declare_parameter(
            'required_controllers',
            ['joint_state_broadcaster', 'joint_trajectory_controller'],
        )
        self.declare_parameter('motion_controller', 'joint_trajectory_controller')
        self.declare_parameter('max_joint_delta_rad', 0.10)
        self.declare_parameter('trajectory_duration_s', 5.0)
        self.declare_parameter(
            'safe_joint_positions_rad', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter(
            'joint_min_rad', [-3.14, -2.24, -3.14, -2.57, -3.14, -2.09, -3.14])
        self.declare_parameter(
            'joint_max_rad', [3.14, 2.24, 3.14, 2.57, 3.14, 2.09, 3.14])
        declare_logging_parameters(self)

        self._joint_state_topic = self.get_parameter('joint_state_topic').value
        self._diagnostics_topic = self.get_parameter('diagnostics_topic').value
        self._controller_ns = str(self.get_parameter('controller_manager_ns').value).rstrip('/')
        self._expected_joints: List[str] = list(self.get_parameter('expected_joints').value)
        self._use_fake_hardware = bool(self.get_parameter('use_fake_hardware').value)
        self._enable_motion = bool(self.get_parameter('enable_motion').value)
        self._motion_controller = str(self.get_parameter('motion_controller').value)
        self._robot_ip = str(self.get_parameter('robot_ip').value)
        self._driver_local = bool(self.get_parameter('start_driver').value)
        self._required_controllers: List[str] = list(
            self.get_parameter('required_controllers').value)

        # ------------------------------------------------------------- logging ----
        # El subsistema de logging se instala ANTES que cualquier interfaz para que los
        # errores de configuración se registren con el nivel y formato definitivos.
        self._log = ThrottledLogger(
            self.get_logger(), float(self.get_parameter('log_throttle_period_s').value))
        self._level = DynamicLogLevel(self, self._log)
        self._transitions = StateTransitionLogger(
            self._log, bool(self.get_parameter('log_state_transitions').value))
        log_logging_banner(self)

        # ------------------------------------------------------- caja negra -------
        self._recorder = FlightRecorder(
            capacity=int(self.get_parameter('flight_recorder_samples').value),
            enabled=bool(self.get_parameter('enable_flight_recorder').value),
        )
        attach_flight_recorder_services(self, self._recorder, self._on_anomaly_changed)

        # ------------------------------------------------- validación de config ---
        self._config_errors = validate_safety_config(
            self._expected_joints,
            list(self.get_parameter('safe_joint_positions_rad').value),
            list(self.get_parameter('joint_min_rad').value),
            list(self.get_parameter('joint_max_rad').value),
            float(self.get_parameter('max_joint_delta_rad').value),
            float(self.get_parameter('trajectory_duration_s').value),
        )
        for error in self._config_errors:
            self.get_logger().error(f'[CONFIGURACIÓN INSEGURA] {error}')
        if self._config_errors:
            self.get_logger().error(
                'El monitor seguirá publicando diagnóstico, pero el movimiento queda '
                'bloqueado hasta corregir config/kinova_connection.yaml.'
            )

        # -------------------------------------------------------------- estado ----
        self._health = LinkHealth(
            expected_joints=self._expected_joints,
            timeout_s=float(self.get_parameter('joint_state_timeout_s').value),
            min_hz=float(self.get_parameter('min_joint_state_hz').value),
            window_samples=int(self.get_parameter('hz_window_samples').value),
            min_span_s=float(self.get_parameter('min_rate_observation_s').value),
        )
        self._controllers: Dict[str, str] = {}
        self._controller_service_ready = False
        self._controller_last_error = 'aún no se ha consultado list_controllers'
        self._controller_request_pending = False
        self._motion_latched_off = False
        self._latch_reason = ''

        # ---------------------------------------------------------- interfaces ----
        diagnostics_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, self._diagnostics_topic, diagnostics_qos)
        self._joint_state_sub = self.create_subscription(
            JointState, self._joint_state_topic, self._on_joint_state, qos_profile_sensor_data)
        self._controller_client = self.create_client(
            ListControllers, f'{self._controller_ns}/list_controllers')
        self._reset_srv = self.create_service(
            Trigger, '~/rehabilitar_movimiento', self._on_reset_latch)

        # --------------------------------------------------- anuncio de estación ---
        # Nadie puede observar desde fuera qué máquina tiene la sesión con el robot: ese
        # tráfico es unicast y el switch no lo replica. Así que esta estación anuncia por
        # broadcast lo que sí puede verificar de sí misma, y el monitor del laboratorio lo
        # muestra. Si algo del anuncio falla, el nodo continúa: no es crítico.
        self._announcer: Optional[StationAnnouncer] = None
        self._announce_timer = None
        if bool(self.get_parameter('announce_station').value):
            self._announcer = StationAnnouncer(
                puerto=int(self.get_parameter('announce_port').value),
                on_error=lambda motivo: self.get_logger().warn(
                    f'[ANUNCIO] {motivo}. El monitor de red no verá esta estación; '
                    f'el diagnóstico ROS 2 no se ve afectado.'
                ),
            )
            self._announce_timer = self.create_timer(
                max(1.0, float(self.get_parameter('announce_period_s').value)),
                self._announce_station,
            )

        diagnostic_period = 1.0 / max(0.01, float(self.get_parameter('diagnostic_rate_hz').value))
        self._diagnostic_timer = self.create_timer(diagnostic_period, self._publish_diagnostics)
        self._controller_timer = self.create_timer(
            max(0.1, float(self.get_parameter('controller_query_period_s').value)),
            self._query_controllers,
        )

        self.add_on_set_parameters_callback(self._on_runtime_parameters)

        self.get_logger().info(
            f'kinova_monitor iniciado | telemetría={self._joint_state_topic} '
            f'(sensor_data/BEST_EFFORT) | diagnóstico={self._diagnostics_topic} (RELIABLE) '
            f"| modo={'FAKE' if self._use_fake_hardware else 'HARDWARE REAL'} "
            f"| movimiento={'HABILITADO' if self._enable_motion else 'BLOQUEADO'}"
        )

    # ------------------------------------------------------------------ tiempo ----
    def _now_s(self) -> float:
        """
        Instante actual en segundos, tomado del reloj monótono del sistema.

        Se usa el reloj monótono y no el de ROS porque las métricas de salud del enlace
        deben seguir siendo válidas aunque el driver publique marcas de tiempo
        desfasadas o el tiempo simulado se detenga.

        :returns: segundos monótonos.
        """
        return time.monotonic()

    # -------------------------------------------------------------- telemetría ----
    def _on_joint_state(self, msg: JointState) -> None:
        """
        Validar y contabilizar un mensaje ``/joint_states``.

        Este callback se ejecuta a la frecuencia del driver (decenas de Hz), por lo que
        todo su registro va limitado en frecuencia: un ``info()`` sin throttling aquí
        saturaría la CPU y degradaría el determinismo del nodo.

        :param msg: mensaje recibido.
        """
        now = self._now_s()
        validation = validate_joint_state(msg.name, msg.position, self._expected_joints)
        was_fresh = self._health.is_fresh(now)
        self._health.update(validation, now)

        if not validation.valid:
            self._log.warn(
                f'[TELEMETRÍA] mensaje descartado: {validation.reason} '
                f'(rechazados={self._health.rejected}/{self._health.received})'
            )
        else:
            self._log.debug(
                f'[TELEMETRÍA] {len(validation.positions)} articulaciones | '
                f'{self._health.hz():.1f} Hz | '
                + ' '.join(f'{k}={v:+.4f}' for k, v in validation.positions.items())
            )
            if not was_fresh and self._health.dropouts > 0:
                self._log.info(
                    f'[RECUPERACIÓN] /joint_states volvió a fluir tras '
                    f'{self._health.dropouts} interrupción(es). El movimiento sigue '
                    f'bloqueado hasta una nueva habilitación explícita.',
                    throttle=False,
                )

        self._recorder.record({
            't': now,
            'valido': validation.valid,
            'hz': self._health.hz(),
            'faltantes': len(validation.missing),
            'posiciones': [validation.positions.get(j, float('nan'))
                           for j in self._expected_joints],
        })

    # ------------------------------------------------------------ controladores ---
    def _query_controllers(self) -> None:
        """
        Consultar ``list_controllers`` de forma asíncrona.

        La indisponibilidad del servicio se refleja en el diagnóstico y **no** termina el
        nodo (RF-04). Tampoco se usa una llamada bloqueante: eso congelaría el ejecutor y
        dejaría de medirse la frecuencia de la telemetría.
        """
        if self._controller_request_pending:
            self._log.debug('[CONTROLADORES] consulta anterior aún en curso; se omite ciclo')
            return
        if not self._controller_client.service_is_ready():
            self._controller_service_ready = False
            self._controller_last_error = (
                f'servicio {self._controller_ns}/list_controllers no disponible'
            )
            self._log.warn(f'[CONTROLADORES] {self._controller_last_error}')
            return

        self._controller_service_ready = True
        self._controller_request_pending = True
        future = self._controller_client.call_async(ListControllers.Request())
        future.add_done_callback(self._on_controllers_response)

    def _on_controllers_response(self, future) -> None:
        """
        Procesar la respuesta de ``list_controllers``.

        :param future: futuro completado de la llamada asíncrona.
        """
        self._controller_request_pending = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - se registra la causa y se continúa
            self._controller_last_error = f'fallo al consultar list_controllers: {exc}'
            self._log.error(f'[CONTROLADORES] {self._controller_last_error}')
            return
        if response is None:
            self._controller_last_error = 'list_controllers devolvió una respuesta vacía'
            self._log.error(f'[CONTROLADORES] {self._controller_last_error}')
            return

        previous = dict(self._controllers)
        self._controllers = {c.name: c.state for c in response.controller}
        self._controller_last_error = ''
        if previous != self._controllers:
            resumen = ', '.join(f'{n}={s}' for n, s in sorted(self._controllers.items()))
            self.get_logger().info(f'[CONTROLADORES] {resumen or "(ninguno cargado)"}')
        else:
            self._log.debug(f'[CONTROLADORES] sin cambios ({len(self._controllers)} cargados)')

    def _controller_state(self) -> Tuple[str, str, List[str]]:
        """
        Evaluar el estado de los controladores requeridos.

        :returns: tupla ``(estado, motivo, faltantes)``.
        """
        if not self._controller_service_ready and not self._controllers:
            return (
                STATE_ERROR,
                self._controller_last_error or 'controller_manager no localizado',
                list(self._required_controllers),
            )
        inactive = [
            name for name in self._required_controllers
            if self._controllers.get(name) != 'active'
        ]
        if inactive:
            detail = ', '.join(
                f"{name}={self._controllers.get(name, 'no cargado')}" for name in inactive)
            return STATE_ERROR, f'controladores no activos: {detail}', inactive
        return STATE_OK, 'todos los controladores requeridos están activos', []

    # --------------------------------------------------------------- movimiento ---
    def _motion_allowed(self) -> Tuple[bool, str]:
        """
        Determinar si el monitor considera habilitado el movimiento.

        :returns: tupla ``(permitido, motivo)``.
        """
        if self._config_errors:
            return False, 'configuración de seguridad inválida'
        if not self._enable_motion:
            return False, 'enable_motion=false (valor seguro por defecto)'
        if self._motion_latched_off:
            return False, f'bloqueo activo: {self._latch_reason}'
        if self._recorder.anomaly_active:
            return False, f'anomalía activa: {self._recorder.anomaly_reason}'
        if not self._health.is_fresh(self._now_s()):
            return False, 'telemetría vencida o ausente'
        if self._controllers.get(self._motion_controller) != 'active':
            return False, f'{self._motion_controller} no está activo'
        return True, 'habilitación explícita vigente y enlace saludable'

    def _latch_motion_off(self, reason: str) -> None:
        """
        Bloquear el movimiento hasta una nueva habilitación explícita.

        :param reason: causa registrada del bloqueo.
        """
        if self._motion_latched_off:
            return
        self._motion_latched_off = True
        self._latch_reason = reason
        self.get_logger().error(
            f'[SEGURIDAD] Movimiento bloqueado por: {reason}. Tras restablecer el enlace, '
            f'rehabilita de forma explícita con: '
            f'ros2 service call /{self.get_name()}/rehabilitar_movimiento std_srvs/srv/Trigger'
        )

    def _on_reset_latch(self, request, response):
        """
        Atender la rehabilitación explícita del movimiento tras un bloqueo.

        :param request: petición ``Trigger`` (sin campos).
        :param response: respuesta a completar.
        :returns: la respuesta completada.
        """
        del request
        if not self._motion_latched_off:
            response.success = True
            response.message = 'no había ningún bloqueo activo'
            return response
        if not self._health.is_fresh(self._now_s()):
            response.success = False
            response.message = (
                'rehabilitación rechazada: la telemetría sigue vencida; restablece primero '
                'el enlace con el driver'
            )
            self.get_logger().error(f'[SEGURIDAD] {response.message}')
            return response
        self._motion_latched_off = False
        previous = self._latch_reason
        self._latch_reason = ''
        response.success = True
        response.message = f'movimiento rehabilitado (bloqueo previo: {previous})'
        self.get_logger().warn(f'[SEGURIDAD] {response.message}')
        return response

    def _on_anomaly_changed(self, active: bool, reason: str) -> None:
        """
        Reaccionar a la inyección o despeje de una anomalía en la caja negra.

        :param active: nuevo estado de la anomalía.
        :param reason: motivo asociado.
        """
        if active:
            self._latch_motion_off(f'anomalía inyectada ({reason})')

    def _on_runtime_parameters(self, params) -> SetParametersResult:
        """
        Aplicar en caliente los parámetros de operación admitidos.

        Rehabilitar ``enable_motion`` cuenta como habilitación explícita y despeja el
        bloqueo por pérdida de enlace, tal como exige el requisito de seguridad 8.

        :param params: parámetros que se intentan fijar.
        :returns: resultado de la validación.
        """
        for param in params:
            if param.name == 'enable_motion':
                self._enable_motion = bool(param.value)
                if self._enable_motion:
                    self._motion_latched_off = False
                    self._latch_reason = ''
                    self.get_logger().warn(
                        '[SEGURIDAD] enable_motion=true: habilitación explícita registrada. '
                        'Verifica espacio despejado y parada de emergencia accesible.'
                    )
                else:
                    self.get_logger().info('[SEGURIDAD] enable_motion=false: movimiento inhibido')
            elif param.name == 'joint_state_timeout_s':
                if param.value <= 0.0:
                    return SetParametersResult(
                        successful=False, reason='joint_state_timeout_s debe ser positivo')
                self._health.timeout_s = float(param.value)
            elif param.name == 'min_joint_state_hz':
                if param.value <= 0.0:
                    return SetParametersResult(
                        successful=False, reason='min_joint_state_hz debe ser positivo')
                self._health.min_hz = float(param.value)
        return SetParametersResult(successful=True)

    # -------------------------------------------------------------- diagnóstico ---
    def _publish_diagnostics(self) -> None:
        """Construir y publicar el diagnóstico consolidado del enlace (RF-05)."""
        now = self._now_s()
        link_state, link_reason, link_action = self._health.classify(now)
        if link_state == STATE_ERROR and self._health.age(now) is not None:
            self._health.note_dropout(link_reason)
            self._latch_motion_off('pérdida de telemetría articular')
        elif link_state == STATE_ERROR:
            self._health.note_error(link_reason)

        ctrl_state, ctrl_reason, ctrl_missing = self._controller_state()
        motion_ok, motion_reason = self._motion_allowed()
        overall = worst_state(link_state, ctrl_state)
        if self._config_errors:
            overall = STATE_ERROR

        self._transitions.update(overall, f'{link_reason} | {ctrl_reason}')

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [
            self._general_status(overall, link_reason, ctrl_reason, motion_ok, motion_reason),
            self._station_status(),
            self._telemetry_status(link_state, link_reason, link_action, now),
            self._controller_status(ctrl_state, ctrl_reason, ctrl_missing),
            self._motion_status(motion_ok, motion_reason),
        ]
        self._diagnostics_pub.publish(array)

        self._log.debug(
            f'[DIAGNÓSTICO] general={overall} enlace={link_state} '
            f'controladores={ctrl_state} movimiento={"OK" if motion_ok else "BLOQUEADO"}',
            throttle=False,
        )

    def _general_status(self, overall, link_reason, ctrl_reason, motion_ok, motion_reason):
        """Construir el estado consolidado del enlace."""
        status = DiagnosticStatus()
        status.level = _DIAGNOSTIC_LEVEL[overall]
        status.name = 'burger_kinova_connection: estado general'
        status.hardware_id = 'kinova_gen3_7dof'
        status.message = f'{overall}: {link_reason}'
        status.values = [
            KeyValue(key='estado_general', value=overall),
            KeyValue(key='modo',
                     value='fake' if self._use_fake_hardware else 'hardware_real'),
            KeyValue(key='movimiento',
                     value='habilitado' if motion_ok else 'bloqueado'),
            KeyValue(key='movimiento_motivo', value=motion_reason),
            KeyValue(key='controladores', value=ctrl_reason),
            KeyValue(key='ultimo_error', value=self._health.last_error or 'ninguno'),
            KeyValue(key='errores_configuracion',
                     value='; '.join(self._config_errors) or 'ninguno'),
            KeyValue(key='nivel_log', value=self._level.level_name),
        ]
        return status

    def _announce_station(self) -> None:
        """
        Emitir el anuncio periódico del rol de esta estación.

        El periodo también actúa como TTL en el receptor: si esta estación se apaga, el
        monitor deja de recibir y su entrada caduca sola, sin necesidad de un mensaje de
        despedida que un apagón nunca llegaría a enviar.
        """
        if self._announcer is None:
            return
        identidad = describir_estacion(self._robot_ip, self._driver_local)
        self._announcer.anunciar(
            construir_anuncio(identidad, self._robot_ip, self.get_name()))

    def _station_status(self):
        """
        Anunciar qué estación es esta y si es la anfitriona del driver.

        ROS 2 no expone en qué máquina corre un nodo, así que con un solo robot y varios
        equipos nadie puede averiguar por introspección quién lo tiene ocupado. Cada
        monitor resuelve su mitad del problema: comprueba **localmente** si su máquina
        mantiene la sesión TCP con la controladora y lo publica aquí. Cualquier estación
        del mismo dominio lee entonces quién es la anfitriona, sin configuración que
        mantener y sin dejar de funcionar cuando mañana sea otro computador.
        """
        info = describir_estacion(self._robot_ip, self._driver_local)
        status = DiagnosticStatus()
        # Informativo, nunca ERROR: ser cliente es el estado normal y esperado.
        status.level = (DiagnosticStatus.OK if info['rol_verificado'] == 'si'
                        else DiagnosticStatus.WARN)
        status.name = 'burger_kinova_connection: identidad de la estación'
        status.hardware_id = 'kinova_gen3_7dof'
        status.message = (
            f"{info['estacion']} ({info['estacion_ip'] or 'ip desconocida'}) — "
            f"{info['rol_estacion']}"
        )
        status.values = [KeyValue(key=k, value=v) for k, v in info.items()]
        status.values.append(KeyValue(key='robot_ip', value=self._robot_ip))
        if self._announcer is None:
            status.values.append(KeyValue(key='anuncio_broadcast', value='desactivado'))
        else:
            status.values.append(KeyValue(
                key='anuncio_broadcast',
                value=f'emitidos={self._announcer.enviados} '
                      f'fallos={self._announcer.fallos}'))
            if self._announcer.ultimo_error:
                status.values.append(KeyValue(
                    key='anuncio_ultimo_error', value=self._announcer.ultimo_error))
        return status

    def _telemetry_status(self, state, reason, action, now):
        """Construir el estado de la telemetría articular."""
        age = self._health.age(now)
        status = DiagnosticStatus()
        status.level = _DIAGNOSTIC_LEVEL[state]
        status.name = 'burger_kinova_connection: telemetría /joint_states'
        status.hardware_id = 'kinova_gen3_7dof'
        status.message = reason
        detected = self._health.detected
        status.values = [
            KeyValue(key='topico', value=str(self._joint_state_topic)),
            KeyValue(key='qos', value='sensor_data (BEST_EFFORT, KEEP_LAST/5)'),
            KeyValue(key='edad_s', value='sin_datos' if age is None else f'{age:.3f}'),
            KeyValue(key='frecuencia_hz', value=f'{self._health.hz():.2f}'),
            KeyValue(key='frecuencia_minima_hz', value=f'{self._health.min_hz:.2f}'),
            KeyValue(key='intervalo_maximo_s', value=f'{self._health.max_gap():.3f}'),
            KeyValue(key='articulaciones_detectadas',
                     value=f'{len(detected)}/{len(self._expected_joints)}'),
            KeyValue(key='articulaciones_faltantes',
                     value=', '.join(self._health.missing) or 'ninguna'),
            KeyValue(key='mensajes_recibidos', value=str(self._health.received)),
            KeyValue(key='mensajes_rechazados', value=str(self._health.rejected)),
            KeyValue(key='interrupciones', value=str(self._health.dropouts)),
            KeyValue(key='caja_negra_muestras',
                     value=f'{len(self._recorder)}/{self._recorder.capacity}'),
            KeyValue(key='accion_recomendada', value=action),
        ]
        return status

    def _controller_status(self, state, reason, missing):
        """Construir el estado de los controladores de ``ros2_control``."""
        status = DiagnosticStatus()
        status.level = _DIAGNOSTIC_LEVEL[state]
        status.name = 'burger_kinova_connection: controladores ros2_control'
        status.hardware_id = 'kinova_gen3_7dof'
        status.message = reason
        status.values = [
            KeyValue(key='servicio', value=f'{self._controller_ns}/list_controllers'),
            KeyValue(key='servicio_disponible',
                     value='si' if self._controller_service_ready else 'no'),
            KeyValue(key='requeridos', value=', '.join(self._required_controllers)),
            KeyValue(key='no_activos', value=', '.join(missing) or 'ninguno'),
            KeyValue(key='ultimo_error', value=self._controller_last_error or 'ninguno'),
        ]
        status.values.extend(
            KeyValue(key=f'controlador.{name}', value=state_name)
            for name, state_name in sorted(self._controllers.items())
        )
        return status

    def _motion_status(self, motion_ok, reason):
        """Construir el estado de la habilitación de movimiento."""
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK if motion_ok else DiagnosticStatus.WARN
        status.name = 'burger_kinova_connection: habilitación de movimiento'
        status.hardware_id = 'kinova_gen3_7dof'
        status.message = (
            'movimiento habilitado' if motion_ok else f'movimiento bloqueado: {reason}')
        status.values = [
            KeyValue(key='enable_motion', value=str(self._enable_motion).lower()),
            KeyValue(key='bloqueo_por_perdida',
                     value=str(self._motion_latched_off).lower()),
            KeyValue(key='bloqueo_motivo', value=self._latch_reason or 'ninguno'),
            KeyValue(key='anomalia_inyectada',
                     value=str(self._recorder.anomaly_active).lower()),
            KeyValue(key='accion_de_rehabilitacion',
                     value=f'ros2 service call /{self.get_name()}/rehabilitar_movimiento '
                           f'std_srvs/srv/Trigger'),
        ]
        return status


def main(args: Optional[List[str]] = None) -> None:
    """
    Punto de entrada del ejecutable ``kinova_monitor``.

    :param args: argumentos de línea de comandos.
    """
    rclpy.init(args=args)
    node = None
    try:
        node = KinovaMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('Interrupción por teclado: cerrando el monitor.')
    except Exception as exc:  # noqa: BLE001 - se registra la causa antes de propagar
        logger = (node.get_logger() if node is not None
                  else rclpy.logging.get_logger('kinova_monitor'))
        logger.fatal(f'El monitor terminó por una excepción no controlada: {exc}')
        raise
    finally:
        # El cierre se protege por separado: una segunda señal (Ctrl+C repetido o el
        # SIGTERM que envía launch tras el SIGINT) no debe convertir un apagado ordenado
        # en un traceback (RF-08: el sistema no se cierra abruptamente).
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


if __name__ == '__main__':
    main()
