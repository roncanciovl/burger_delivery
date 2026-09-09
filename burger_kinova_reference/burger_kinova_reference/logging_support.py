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

"""
Subsistema de logging del package, aplicando la teoría del taller de rosbag2/logging.

Este módulo materializa, en código ejecutable, los conceptos descritos en
``docs/TEORIA_LOGGING_ROS2.md``:

1. **Los cinco niveles de severidad** (``DEBUG`` 10, ``INFO`` 20, ``WARN`` 30,
   ``ERROR`` 40, ``FATAL`` 50) con un criterio explícito de cuándo usar cada uno
   en el contexto del enlace con el Kinova Gen3.
2. **Cambio dinámico del nivel en caliente**, sin detener ni recompilar el nodo,
   mediante el parámetro ``log_level`` y un callback de parámetros. Es el
   equivalente programático de ``ros2 param set /kinova_monitor log_level debug``.
3. **Throttling**: un ``info()`` dentro de un callback de sensor a 40 Hz satura la
   CPU y degrada el determinismo. :class:`ThrottledLogger` centraliza el periodo
   configurable y delega en ``throttle_duration_sec`` de ``rclpy``.
4. **Registro por transición de estado**: ``/rosout`` debe contener la línea de
   tiempo del incidente, no una repetición del mismo mensaje. :class:`StateTransitionLogger`
   sólo emite cuando el estado observado cambia.
5. **Trazabilidad del entorno**: el formato de consola (``RCUTILS_CONSOLE_OUTPUT_FORMAT``),
   el coloreado y el directorio de logs rotativos se reportan al arrancar, de modo
   que cualquier evidencia entregada sea reproducible.

Los tres destinos (*sinks*) del subsistema ``rcutils``/``spdlog`` son:

* la consola (``stdout``/``stderr``, con formato y color configurables),
* los archivos rotativos en ``~/.ros/log/``,
* y el tópico agregado ``/rosout`` (``rcl_interfaces/msg/Log``), que alimenta a
  ``rqt_console`` y puede grabarse con ``rosbag2``.
"""

import os
import sys
import time
from typing import Callable, Dict, Optional, Tuple

from rcl_interfaces.msg import SetParametersResult
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.parameter import Parameter


#: Traducción de nombre textual a severidad de ``rclpy``. Se aceptan mayúsculas o
#: minúsculas porque ``ros2 param set`` y ``--ros-args --log-level`` se escriben de
#: ambas formas en la práctica.
SEVERITY_BY_NAME: Dict[str, LoggingSeverity] = {
    'debug': LoggingSeverity.DEBUG,
    'info': LoggingSeverity.INFO,
    'warn': LoggingSeverity.WARN,
    'warning': LoggingSeverity.WARN,
    'error': LoggingSeverity.ERROR,
    'fatal': LoggingSeverity.FATAL,
}

#: Valor numérico de cada nivel, tal como viaja en ``rcl_interfaces/msg/Log.level``.
SEVERITY_VALUE: Dict[str, int] = {
    'debug': 10,
    'info': 20,
    'warn': 30,
    'error': 40,
    'fatal': 50,
}

#: Criterio de uso de cada nivel dentro de este package. Se documenta en código para
#: que la revisión por pares pueda verificar que un mensaje está en el nivel correcto.
SEVERITY_POLICY: Dict[str, str] = {
    'debug': 'Trazas por ciclo: edad exacta de /joint_states, deltas por articulación, '
             'volcado de la caja negra. Desactivado en operación normal.',
    'info': 'Progreso verificable: nodo iniciado, controladores activos, meta aceptada.',
    'warn': 'Condición inesperada pero recuperable: frecuencia por debajo del mínimo, '
            'controlador aún no activo, servicio todavía no disponible.',
    'error': 'Falla funcional que impide completar la tarea: telemetría vencida, meta '
             'rechazada, servidor de acción ausente, límite articular excedido.',
    'fatal': 'Condición crítica de seguridad que exige intervención humana inmediata '
             '(parada de emergencia). Este package NO la usa para recuperarse solo.',
}

#: Formato de consola recomendado para el laboratorio. Incluye archivo y línea, lo que
#: permite localizar de inmediato el origen de una alerta.
RECOMMENDED_CONSOLE_FORMAT = (
    '[{severity}] [{time}] [{name} -> {function_name}:{line_number}]: {message}'
)


def severity_from_name(name: str, default: LoggingSeverity = LoggingSeverity.INFO):
    """
    Convertir un nombre de nivel a :class:`LoggingSeverity`.

    :param name: nombre del nivel (``debug``, ``INFO``, ``warn``...).
    :param default: valor devuelto cuando el nombre no es reconocido.
    :returns: la severidad correspondiente, o ``default`` si el nombre es inválido.
    """
    if not isinstance(name, str):
        return default
    return SEVERITY_BY_NAME.get(name.strip().lower(), default)


def is_valid_severity(name: str) -> bool:
    """Indicar si ``name`` corresponde a un nivel de severidad conocido."""
    return isinstance(name, str) and name.strip().lower() in SEVERITY_BY_NAME


def describe_logging_environment() -> Dict[str, str]:
    """
    Reportar la configuración efectiva del subsistema de logging.

    Se consulta el entorno real del proceso para que la evidencia entregada indique
    con qué formato y coloreado se produjo cada captura de consola.

    :returns: diccionario con las variables relevantes y el directorio de logs.
    """
    return {
        'RCUTILS_CONSOLE_OUTPUT_FORMAT': os.environ.get(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '(por defecto) [{severity}] [{name}]: {message}'
        ),
        'RCUTILS_COLORIZED_OUTPUT': os.environ.get('RCUTILS_COLORIZED_OUTPUT', '(auto)'),
        'RCUTILS_LOGGING_BUFFERED_STREAM': os.environ.get(
            'RCUTILS_LOGGING_BUFFERED_STREAM', '(no definido)'
        ),
        'ROS_LOG_DIR': os.environ.get(
            'ROS_LOG_DIR', os.path.join(os.path.expanduser('~'), '.ros', 'log')
        ),
        'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0 (por defecto)'),
        'RMW_IMPLEMENTATION': os.environ.get('RMW_IMPLEMENTATION', '(por defecto del sistema)'),
    }


class ThrottledLogger:
    """
    Envoltura de logger que aplica limitación de frecuencia por sitio de llamada.

    Regla de oro del taller: *nunca* registrar sin límite dentro de un callback de
    sensor. Esta clase mantiene un periodo único y configurable para todo el nodo, de
    modo que la verbosidad se ajusta desde el YAML y no editando cada llamada.

    Los métodos aceptan ``throttle=False`` para los eventos que deben aparecer siempre
    (transiciones de estado, resultados de una meta, fallas de seguridad).

    **Por qué el límite se implementa aquí y no con ``throttle_duration_sec``.**
    ``rclpy`` cachea un contexto de logging por *sitio de llamada* (archivo, función y
    línea) y prohíbe que ese sitio cambie de severidad, de conjunto de filtros o de
    parámetros del filtro entre llamadas. Una envoltura genérica delegando en
    ``throttle_duration_sec`` reutilizaría el mismo sitio para todos los niveles y
    lanzaría ``ValueError: Logger severity cannot be changed between calls``, y además
    impediría ajustar ``log_throttle_period_s`` en caliente. Manteniendo el estado del
    límite en esta clase se conserva la semántica de ROS —un cubo de throttling por
    sitio de llamada— y el periodo sí puede cambiarse en tiempo de ejecución.
    """

    def __init__(self, logger: RcutilsLogger, period_s: float = 2.0):
        """
        Construir la envoltura.

        :param logger: logger del nodo (``node.get_logger()``).
        :param period_s: periodo mínimo entre dos mensajes del mismo sitio de llamada.
        """
        self._logger = logger
        self._period_s = max(0.0, float(period_s))
        self._last_emit: Dict[Tuple[str, str, int], float] = {}

    @property
    def period_s(self) -> float:
        """Periodo de throttling vigente, en segundos."""
        return self._period_s

    def set_period(self, period_s: float) -> None:
        """Actualizar el periodo de throttling en caliente."""
        self._period_s = max(0.0, float(period_s))

    @property
    def raw(self) -> RcutilsLogger:
        """Logger subyacente, para casos que necesiten la API completa de ``rclpy``."""
        return self._logger

    def _allow(self, level: str, throttle: bool, key: Optional[str]) -> bool:
        """
        Decidir si corresponde emitir el mensaje según el límite de frecuencia.

        :param level: nivel del mensaje, para que cada severidad tenga su propio cubo.
        :param throttle: ``False`` desactiva el límite para esta llamada.
        :param key: identificador explícito del cubo; si es ``None`` se usa el sitio de
            llamada real (archivo y línea), replicando la semántica de ROS 2.
        :returns: ``True`` si el mensaje debe registrarse.
        """
        if not throttle or self._period_s <= 0.0:
            return True
        if key is None:
            # Marco 0 = _allow, 1 = debug/info/warn/error/fatal, 2 = quien registra.
            frame = sys._getframe(2)
            bucket = (level, frame.f_code.co_filename, frame.f_lineno)
        else:
            bucket = (level, key, 0)
        now = time.monotonic()
        last = self._last_emit.get(bucket)
        if last is not None and (now - last) < self._period_s:
            return False
        self._last_emit[bucket] = now
        return True

    def debug(self, message: str, throttle: bool = True, key: Optional[str] = None) -> None:
        """Registrar una traza de ciclo (nivel ``DEBUG``)."""
        if self._allow('debug', throttle, key):
            self._logger.debug(message)

    def info(self, message: str, throttle: bool = False, key: Optional[str] = None) -> None:
        """Registrar progreso verificable (nivel ``INFO``)."""
        if self._allow('info', throttle, key):
            self._logger.info(message)

    def warn(self, message: str, throttle: bool = True, key: Optional[str] = None) -> None:
        """Registrar una condición recuperable (nivel ``WARN``)."""
        if self._allow('warn', throttle, key):
            self._logger.warn(message)

    def error(self, message: str, throttle: bool = True, key: Optional[str] = None) -> None:
        """Registrar una falla funcional (nivel ``ERROR``)."""
        if self._allow('error', throttle, key):
            self._logger.error(message)

    def fatal(self, message: str, throttle: bool = False, key: Optional[str] = None) -> None:
        """Registrar una condición crítica de seguridad (nivel ``FATAL``)."""
        if self._allow('fatal', throttle, key):
            self._logger.fatal(message)


class StateTransitionLogger:
    """
    Emitir a ``/rosout`` únicamente los cambios de estado del enlace.

    Un monitor que repite ``ERROR: sin telemetría`` a 1 Hz produce miles de líneas
    inútiles y esconde el instante real del fallo. Registrando sólo la transición,
    ``rqt_console`` y una bolsa de ``/rosout`` quedan con la cronología exacta del
    incidente: cuándo se degradó, cuándo se perdió y cuándo se recuperó.
    """

    def __init__(self, logger: ThrottledLogger, enabled: bool = True):
        """
        Construir el registrador de transiciones.

        :param logger: logger envuelto del nodo.
        :param enabled: si es ``False``, la clase no emite nada (útil en pruebas).
        """
        self._logger = logger
        self._enabled = enabled
        self._previous: Optional[str] = None

    @property
    def previous(self) -> Optional[str]:
        """Último estado registrado, o ``None`` si aún no hubo ninguno."""
        return self._previous

    def update(self, state: str, detail: str = '') -> bool:
        """
        Notificar el estado actual y registrar sólo si cambió.

        :param state: estado observado (``OK``, ``WARN``, ``ERROR``...).
        :param detail: explicación breve que acompaña la transición.
        :returns: ``True`` si hubo transición (y por tanto se registró).
        """
        if state == self._previous:
            return False
        previous = self._previous
        self._previous = state
        if not self._enabled:
            return True
        origin = previous if previous is not None else 'INICIO'
        message = f'[TRANSICIÓN] {origin} -> {state}'
        if detail:
            message = f'{message} | {detail}'
        if state == 'ERROR':
            self._logger.error(message, throttle=False)
        elif state == 'WARN':
            self._logger.warn(message, throttle=False)
        else:
            self._logger.info(message, throttle=False)
        return True


class DynamicLogLevel:
    """
    Permitir cambiar el nivel de log del nodo en tiempo de ejecución.

    En una misión real no se puede detener el robot y recompilar sólo para añadir una
    traza. Con esta clase el nivel se cambia en caliente::

        ros2 param set /kinova_monitor log_level debug
        ros2 param set /kinova_monitor log_throttle_period_s 0.5

    lo que equivale, sin salir del grafo ROS 2, a ``rqt_logger_level`` o al servicio
    ``/kinova_monitor/set_logger_levels``.
    """

    def __init__(
        self,
        node: Node,
        throttled: ThrottledLogger,
        on_change: Optional[Callable[[str], None]] = None,
    ):
        """
        Instalar el control dinámico sobre un nodo ya construido.

        Los parámetros ``log_level`` y ``log_throttle_period_s`` deben estar declarados
        antes de llamar a este constructor.

        :param node: nodo al que se aplica el nivel.
        :param throttled: envoltura de logging del nodo.
        :param on_change: callback opcional invocado tras un cambio de nivel válido.
        """
        self._node = node
        self._throttled = throttled
        self._on_change = on_change
        self._level_name = str(
            node.get_parameter('log_level').get_parameter_value().string_value or 'info'
        ).lower()
        self.apply(self._level_name)
        node.add_on_set_parameters_callback(self._on_set_parameters)

    @property
    def level_name(self) -> str:
        """Nombre del nivel vigente."""
        return self._level_name

    def apply(self, name: str) -> bool:
        """
        Aplicar un nivel al logger del nodo.

        :param name: nombre del nivel deseado.
        :returns: ``True`` si el nombre era válido y se aplicó.
        """
        if not is_valid_severity(name):
            return False
        normalized = name.strip().lower()
        self._node.get_logger().set_level(severity_from_name(normalized))
        self._level_name = normalized
        return True

    def _on_set_parameters(self, params) -> SetParametersResult:
        """Validar y aplicar cambios de parámetros de logging en caliente."""
        for param in params:
            if param.name == 'log_level':
                if param.type_ != Parameter.Type.STRING:
                    return SetParametersResult(
                        successful=False, reason='log_level debe ser una cadena'
                    )
                if not is_valid_severity(param.value):
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            f"log_level inválido: '{param.value}'. "
                            f'Valores aceptados: {sorted(SEVERITY_VALUE)}'
                        ),
                    )
                self.apply(param.value)
                self._node.get_logger().info(
                    f'Nivel de log cambiado en caliente a {self._level_name.upper()} '
                    f'({SEVERITY_VALUE[self._level_name]}) — '
                    f'{SEVERITY_POLICY[self._level_name]}'
                )
                if self._on_change is not None:
                    self._on_change(self._level_name)
            elif param.name == 'log_throttle_period_s':
                if param.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False, reason='log_throttle_period_s debe ser double'
                    )
                if param.value < 0.0:
                    return SetParametersResult(
                        successful=False, reason='log_throttle_period_s no puede ser negativo'
                    )
                self._throttled.set_period(param.value)
                self._node.get_logger().info(
                    f'Periodo de throttling de logs ajustado a {param.value:.2f} s'
                )
        return SetParametersResult(successful=True)


def declare_logging_parameters(node: Node) -> None:
    """
    Declarar los parámetros del subsistema de logging con sus valores seguros.

    :param node: nodo sobre el que se declaran los parámetros.
    """
    node.declare_parameter('log_level', 'info')
    node.declare_parameter('log_throttle_period_s', 2.0)
    node.declare_parameter('enable_flight_recorder', True)
    node.declare_parameter('flight_recorder_samples', 200)
    node.declare_parameter('log_state_transitions', True)


def log_logging_banner(node: Node) -> None:
    """
    Registrar, al arrancar, la configuración de logging efectiva del proceso.

    Sin este banner una captura de consola no es reproducible: no se sabe qué formato,
    qué nivel ni qué dominio DDS produjeron la evidencia.

    :param node: nodo que emite el banner.
    """
    logger = node.get_logger()
    env = describe_logging_environment()
    logger.info('── Subsistema de logging ROS 2 (rcutils + spdlog) ──')
    logger.info(f"  Nivel inicial      : {node.get_parameter('log_level').value}")
    logger.info(f"  Throttle por defecto: {node.get_parameter('log_throttle_period_s').value} s")
    logger.info(f"  Formato de consola : {env['RCUTILS_CONSOLE_OUTPUT_FORMAT']}")
    logger.info(f"  Logs rotativos en  : {env['ROS_LOG_DIR']}")
    logger.info(f"  ROS_DOMAIN_ID      : {env['ROS_DOMAIN_ID']}")
    logger.info(f"  RMW                : {env['RMW_IMPLEMENTATION']}")
    logger.info('  Destinos: consola, archivos rotativos y tópico /rosout (rqt_console).')
    logger.debug(
        'Traza DEBUG activa: si lees esta línea, el nivel del nodo es DEBUG. '
        'Cámbialo en caliente con: ros2 param set <nodo> log_level info'
    )
