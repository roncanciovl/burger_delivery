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
Patrón *Flight Recorder* (caja negra) aplicado al enlace con el Kinova Gen3.

En robótica industrial y espacial el robot mantiene continuamente un **búfer circular
en RAM** con los últimos segundos de telemetría. Grabar a disco de forma permanente
sería inviable (satura almacenamiento y red DDS), pero cuando ocurre una anomalía
crítica el sistema vuelca ese búfer para el análisis *post-mortem*.

Este módulo implementa ese patrón con dos piezas separadas a propósito:

* :class:`FlightRecorder`, lógica pura sin dependencias de ROS 2, que puede probarse
  unitariamente sin levantar un grafo (ver ``test/test_flight_recorder.py``).
* :func:`attach_flight_recorder_services`, que expone el volcado y la inyección
  controlada de anomalías como servicios ROS 2 estándar.

Secuencia de uso durante una práctica::

    # 1. Operación nominal: el búfer se llena en RAM, nada se escribe a disco.
    # 2. Ocurre (o se inyecta) la anomalía; el nodo emite un ERROR en /rosout.
    ros2 service call /kinova_monitor/trigger_anomaly std_srvs/srv/SetBool "{data: true}"
    # 3. Se sube la verbosidad y se vuelca el historial previo al fallo.
    ros2 param set /kinova_monitor log_level debug
    ros2 service call /kinova_monitor/dump_flight_recorder std_srvs/srv/Trigger
"""

from collections import deque
from typing import Any, Callable, Deque, Dict, List, Optional

from std_srvs.srv import SetBool, Trigger


class FlightRecorder:
    """
    Búfer circular en memoria con las últimas ``capacity`` muestras de telemetría.

    Es deliberadamente independiente de ROS 2: recibe diccionarios ya formados y no
    conoce tipos de mensaje. Eso permite probar el desbordamiento, el volcado y el
    marcado de anomalías sin ``rclpy``.
    """

    def __init__(self, capacity: int = 200, enabled: bool = True):
        """
        Construir el búfer.

        :param capacity: número máximo de muestras retenidas (mínimo 1).
        :param enabled: si es ``False``, :meth:`record` descarta las muestras.
        """
        self._capacity = max(1, int(capacity))
        self._enabled = bool(enabled)
        self._buffer: Deque[Dict[str, Any]] = deque(maxlen=self._capacity)
        self._anomaly_active = False
        self._anomaly_reason = ''
        self._dumps = 0
        self._dropped = 0

    @property
    def capacity(self) -> int:
        """Capacidad máxima del búfer."""
        return self._capacity

    @property
    def enabled(self) -> bool:
        """Indicar si el búfer está aceptando muestras."""
        return self._enabled

    @property
    def anomaly_active(self) -> bool:
        """Indicar si hay una anomalía marcada como activa."""
        return self._anomaly_active

    @property
    def anomaly_reason(self) -> str:
        """Motivo de la última anomalía activada."""
        return self._anomaly_reason

    @property
    def dropped(self) -> int:
        """Número de muestras que salieron del búfer por desbordamiento circular."""
        return self._dropped

    @property
    def dumps(self) -> int:
        """Número de volcados solicitados desde que arrancó el nodo."""
        return self._dumps

    def __len__(self) -> int:
        """Número de muestras actualmente retenidas."""
        return len(self._buffer)

    def record(self, sample: Dict[str, Any]) -> bool:
        """
        Añadir una muestra al búfer circular.

        :param sample: diccionario con la telemetría del instante.
        :returns: ``True`` si la muestra se almacenó.
        """
        if not self._enabled:
            return False
        if len(self._buffer) == self._capacity:
            self._dropped += 1
        self._buffer.append(dict(sample))
        return True

    def set_anomaly(self, active: bool, reason: str = '') -> None:
        """
        Marcar o limpiar el estado de anomalía.

        No se limpia sola: el requisito de seguridad prohíbe la recuperación automática
        de fallas, así que la desactivación es siempre una acción humana explícita.

        :param active: nuevo estado de la anomalía.
        :param reason: descripción del motivo.
        """
        self._anomaly_active = bool(active)
        self._anomaly_reason = reason if active else ''

    def snapshot(self, last_n: Optional[int] = None) -> List[Dict[str, Any]]:
        """
        Obtener una copia de las muestras retenidas, de la más antigua a la más reciente.

        :param last_n: si se indica, devuelve sólo las ``last_n`` más recientes.
        :returns: lista de muestras.
        """
        samples = [dict(sample) for sample in self._buffer]
        if last_n is not None and last_n >= 0:
            samples = samples[-last_n:] if last_n else []
        return samples

    def clear(self) -> None:
        """Vaciar el búfer sin alterar los contadores históricos."""
        self._buffer.clear()

    def dump_lines(self, last_n: Optional[int] = None) -> List[str]:
        """
        Formatear el contenido del búfer como líneas legibles para el nivel ``DEBUG``.

        :param last_n: número de muestras más recientes a incluir (todas si es ``None``).
        :returns: lista de líneas de texto listas para registrar.
        """
        self._dumps += 1
        samples = self.snapshot(last_n)
        header = (
            f'[FLIGHT RECORDER] volcado #{self._dumps} | muestras={len(samples)}/'
            f'{self._capacity} | descartadas={self._dropped} | '
            f"anomalía={'SÍ' if self._anomaly_active else 'no'}"
        )
        if self._anomaly_active and self._anomaly_reason:
            header = f'{header} | motivo={self._anomaly_reason}'
        lines = [header]
        for index, sample in enumerate(samples):
            fields = ' '.join(f'{key}={_fmt(value)}' for key, value in sample.items())
            lines.append(f'  [{index:04d}] {fields}')
        if not samples:
            lines.append('  (búfer vacío: el nodo aún no ha recibido telemetría)')
        return lines


def _fmt(value: Any) -> str:
    """Formatear un valor de telemetría de forma compacta y estable."""
    if isinstance(value, float):
        return f'{value:.5f}'
    if isinstance(value, (list, tuple)):
        return '[' + ','.join(_fmt(item) for item in value) + ']'
    return str(value)


def attach_flight_recorder_services(
    node,
    recorder: FlightRecorder,
    on_anomaly: Optional[Callable[[bool, str], None]] = None,
):
    """
    Exponer la caja negra como servicios ROS 2 del nodo.

    Se crean dos servicios en el namespace privado del nodo:

    * ``~/dump_flight_recorder`` (``std_srvs/srv/Trigger``): vuelca el búfer en nivel
      ``DEBUG`` y devuelve un resumen en la respuesta.
    * ``~/trigger_anomaly`` (``std_srvs/srv/SetBool``): marca o limpia una anomalía
      inyectada, para ensayar el procedimiento de análisis sin dañar el robot.

    :param node: nodo ROS 2 propietario de los servicios.
    :param recorder: búfer circular a exponer.
    :param on_anomaly: callback opcional notificado al cambiar el estado de anomalía.
    :returns: tupla ``(servicio_dump, servicio_anomalía)``.
    """
    def _dump_callback(request, response):
        del request
        lines = recorder.dump_lines()
        for line in lines:
            node.get_logger().debug(line)
        node.get_logger().info(
            f'Volcado de caja negra solicitado: {len(recorder)} muestras enviadas al '
            f'nivel DEBUG. Si no las ves, sube la verbosidad con: '
            f'ros2 param set /{node.get_name()} log_level debug'
        )
        response.success = True
        response.message = lines[0]
        return response

    def _anomaly_callback(request, response):
        reason = 'anomalía inyectada manualmente para ensayo de diagnóstico'
        recorder.set_anomaly(request.data, reason)
        if request.data:
            node.get_logger().error(
                f'[FALLA INYECTADA] {reason}. El movimiento queda bloqueado hasta una '
                f'nueva habilitación explícita.'
            )
        else:
            node.get_logger().warn(
                '[FALLA INYECTADA] Anomalía despejada por decisión del operador.'
            )
        if on_anomaly is not None:
            on_anomaly(bool(request.data), reason if request.data else '')
        response.success = True
        response.message = (
            f"anomalía {'activada' if request.data else 'despejada'}; "
            f'búfer con {len(recorder)} muestras'
        )
        return response

    dump_srv = node.create_service(Trigger, '~/dump_flight_recorder', _dump_callback)
    anomaly_srv = node.create_service(SetBool, '~/trigger_anomaly', _anomaly_callback)
    return dump_srv, anomaly_srv
