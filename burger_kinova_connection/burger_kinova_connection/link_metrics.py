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
Métricas y clasificación de salud del enlace ``/joint_states``.

Toda la lógica de este módulo es **pura**: no importa ``rclpy`` ni tipos de mensaje.
Recibe listas de nombres, posiciones y marcas de tiempo en segundos, y devuelve
resultados deterministas. Gracias a eso las pruebas unitarias de validación de
mensajes, de estimación de frecuencia y de vencimiento por *timeout* se ejecutan con
``colcon test`` sin necesidad de un robot ni de un grafo ROS 2 activo.

Clasificación de la salud del enlace (RF-03):

============  =========================================================================
Estado        Condición
============  =========================================================================
``OK``        Telemetría fresca, las articulaciones esperadas presentes y frecuencia ≥ mínima.
``WARN``      Enlace vivo pero degradado: frecuencia baja, o mensajes rechazados.
``ERROR``     Sin telemetría, telemetría vencida o articulaciones faltantes.
============  =========================================================================
"""

from collections import deque
import math
from typing import Deque, Dict, List, Optional, Sequence, Tuple

#: Estados posibles del enlace, en orden de gravedad creciente.
STATE_OK = 'OK'
STATE_WARN = 'WARN'
STATE_ERROR = 'ERROR'

_SEVERITY_ORDER = {STATE_OK: 0, STATE_WARN: 1, STATE_ERROR: 2}


def worst_state(*states: str) -> str:
    """
    Devolver el estado más grave de los recibidos.

    :param states: estados a comparar.
    :returns: el estado de mayor severidad, o ``OK`` si no se pasa ninguno.
    """
    if not states:
        return STATE_OK
    return max(states, key=lambda state: _SEVERITY_ORDER.get(state, 0))


class JointStateValidation:
    """Resultado de validar un mensaje ``sensor_msgs/msg/JointState``."""

    def __init__(
        self,
        valid: bool,
        reason: str = '',
        positions: Optional[Dict[str, float]] = None,
        missing: Optional[List[str]] = None,
    ):
        """
        Construir el resultado.

        :param valid: ``True`` si el mensaje es utilizable.
        :param reason: motivo del rechazo, vacío cuando es válido.
        :param positions: mapa ``nombre -> posición`` de las articulaciones esperadas.
        :param missing: articulaciones esperadas que no venían en el mensaje.
        """
        self.valid = valid
        self.reason = reason
        self.positions = positions or {}
        self.missing = missing or []

    def __repr__(self) -> str:
        """Representación compacta útil en los mensajes de las pruebas."""
        return (
            f'JointStateValidation(valid={self.valid}, reason={self.reason!r}, '
            f'missing={self.missing})'
        )


def validate_joint_state(
    names: Sequence[str],
    positions: Sequence[float],
    expected_joints: Sequence[str],
    max_abs_rad: Optional[float] = None,
) -> JointStateValidation:
    """
    Validar un mensaje de estado articular sin depender del orden del arreglo.

    Se rechaza el mensaje cuando (RF-03):

    * no trae nombres,
    * los arreglos de nombres y posiciones son incoherentes en longitud,
    * alguna posición de una articulación esperada no es un número finito,
    * o alguna posición es finita pero físicamente imposible.

    El último caso no es teórico. Con el robot real se observó ``joint_7`` reportando
    ``1.12e+277`` rad de forma constante, con par exactamente cero, porque el brazo
    anunciaba seis actuadores mientras el driver se lanzó con ``dof:=7``: la casilla de
    esa articulación nunca se escribía y conservaba memoria sin inicializar. Como
    ``1.12e+277`` **es** un número finito, comprobar sólo ``isfinite`` daba el enlace por
    saludable mientras una de las siete articulaciones era basura.

    Un mensaje al que sólo le faltan articulaciones **sí** se considera inválido para
    comandar, pero se reporta con la lista concreta de faltantes para que el
    diagnóstico sea accionable.

    :param names: nombres de articulación tal como llegan en el mensaje.
    :param positions: posiciones correspondientes, en el mismo orden que ``names``.
    :param expected_joints: articulaciones que deben estar presentes.
    :param max_abs_rad: magnitud máxima plausible de una posición articular. Si se
        indica, una posición cuyo valor absoluto la supere invalida el mensaje. Un
        límite generoso basta: sirve para distinguir telemetría de basura, no para
        sustituir los límites articulares aprobados de ``joint_min_rad``/``joint_max_rad``.
    :returns: el resultado de la validación.
    """
    names = list(names)
    positions = list(positions)
    expected = list(expected_joints)

    if not names:
        return JointStateValidation(False, 'mensaje sin nombres de articulación',
                                    missing=expected)
    if len(names) != len(positions):
        return JointStateValidation(
            False,
            f'arreglos incoherentes: {len(names)} nombres frente a {len(positions)} '
            f'posiciones',
            missing=expected,
        )

    by_name = dict(zip(names, positions))
    missing = [joint for joint in expected if joint not in by_name]
    if missing:
        return JointStateValidation(
            False,
            f"articulaciones faltantes: {', '.join(missing)}",
            positions={j: by_name[j] for j in expected if j in by_name},
            missing=missing,
        )

    found = {joint: float(by_name[joint]) for joint in expected}
    non_finite = [joint for joint, value in found.items() if not math.isfinite(value)]
    if non_finite:
        return JointStateValidation(
            False,
            f"posiciones no finitas (NaN/inf) en: {', '.join(non_finite)}",
            positions=found,
        )

    if max_abs_rad is not None:
        implausibles = {j: v for j, v in found.items() if abs(v) > max_abs_rad}
        if implausibles:
            detalle = ', '.join(f'{j}={v:.3g}' for j, v in implausibles.items())
            return JointStateValidation(
                False,
                f'posiciones físicamente imposibles (|q| > {max_abs_rad:g} rad): '
                f'{detalle}. Suele indicar que el driver expone más articulaciones de '
                f'las que el robot reporta, y esa casilla nunca se escribe',
                positions=found,
            )

    return JointStateValidation(True, '', positions=found, missing=[])


class RateEstimator:
    """
    Estimador de frecuencia por ventana deslizante de marcas de tiempo.

    Se usa la media sobre la ventana en lugar del inverso del último intervalo porque
    un único mensaje retrasado no debe declarar degradado un enlace sano; a la vez, una
    caída sostenida sí baja la media y se detecta.
    """

    def __init__(self, window_samples: int = 50):
        """
        Construir el estimador.

        :param window_samples: número de marcas de tiempo retenidas (mínimo 2).
        """
        self._window = max(2, int(window_samples))
        self._stamps: Deque[float] = deque(maxlen=self._window)

    @property
    def samples(self) -> int:
        """Número de marcas de tiempo actualmente en la ventana."""
        return len(self._stamps)

    def add(self, stamp_s: float) -> None:
        """Registrar la llegada de un mensaje en el instante ``stamp_s`` (segundos)."""
        self._stamps.append(float(stamp_s))

    def reset(self) -> None:
        """Descartar la ventana, por ejemplo tras una interrupción del enlace."""
        self._stamps.clear()

    def hz(self) -> float:
        """
        Estimar la frecuencia media de llegada.

        :returns: frecuencia en Hz, o ``0.0`` si aún no hay datos suficientes.
        """
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        if span <= 0.0:
            return 0.0
        return (len(self._stamps) - 1) / span

    def span(self) -> float:
        """
        Devolver el tiempo real cubierto por la ventana.

        :returns: segundos entre la primera y la última marca, o ``0.0`` si no hay
            datos suficientes.
        """
        if len(self._stamps) < 2:
            return 0.0
        return max(0.0, self._stamps[-1] - self._stamps[0])

    def max_gap(self) -> float:
        """
        Devolver el mayor intervalo entre mensajes consecutivos de la ventana.

        :returns: intervalo en segundos, o ``0.0`` si no hay datos suficientes.
        """
        if len(self._stamps) < 2:
            return 0.0
        stamps = list(self._stamps)
        return max(b - a for a, b in zip(stamps, stamps[1:]))


class LinkHealth:
    """
    Acumulador del estado del enlace de telemetría articular.

    Mantiene la última recepción válida, la frecuencia estimada, el número de
    interrupciones y el último error observado, y traduce todo eso a un estado
    ``OK``/``WARN``/``ERROR`` con una acción recomendada.
    """

    def __init__(
        self,
        expected_joints: Sequence[str],
        timeout_s: float = 1.0,
        min_hz: float = 20.0,
        window_samples: int = 50,
        min_span_s: float = 0.5,
        max_abs_rad: Optional[float] = None,
    ):
        """
        Construir el acumulador.

        :param expected_joints: articulaciones que deben estar presentes.
        :param timeout_s: edad máxima tolerada del último mensaje válido.
        :param min_hz: frecuencia mínima aceptada antes de declarar degradación.
        :param window_samples: tamaño de la ventana del estimador de frecuencia.
        :param max_abs_rad: magnitud máxima plausible de una posición articular; se
            transporta para quien construya la validación a partir de este acumulador.
        :param min_span_s: tiempo real mínimo de observación continua antes de creer la
            frecuencia estimada. Al suscribirse, DDS entrega de golpe los mensajes ya
            encolados y la ventana se llena en microsegundos, produciendo estimaciones
            absurdas (miles de Hz). Se mide el tiempo transcurrido desde el primer
            mensaje válido del tramo, no el lapso de la ventana: con una ventana de N
            muestras a alta frecuencia ese lapso se satura por debajo del umbral y el
            enlace quedaría marcado como degradado para siempre.
        """
        self.expected_joints = list(expected_joints)
        self.timeout_s = float(timeout_s)
        self.min_hz = float(min_hz)
        self.max_abs_rad = max_abs_rad
        self._rate = RateEstimator(window_samples)
        self._min_span_s = max(0.0, float(min_span_s))
        self._last_valid_s: Optional[float] = None
        self._first_valid_s: Optional[float] = None
        self._last_positions: Dict[str, float] = {}
        self._missing: List[str] = list(expected_joints)
        self._received = 0
        self._rejected = 0
        self._dropouts = 0
        self._in_dropout = True
        self._last_error = ''

    # ------------------------------------------------------------------ ingesta ----
    def update(self, validation: JointStateValidation, now_s: float) -> None:
        """
        Incorporar el resultado de validar un mensaje recién recibido.

        :param validation: resultado de :func:`validate_joint_state`.
        :param now_s: instante de recepción, en segundos monótonos.
        """
        self._received += 1
        if not validation.valid:
            self._rejected += 1
            self._missing = list(validation.missing)
            self._last_error = validation.reason
            return
        if self._in_dropout:
            # Arranque o recuperación tras una interrupción: la ventana anterior ya no
            # es representativa, y el tramo de observación empieza de cero.
            self._rate.reset()
            self._first_valid_s = now_s
        self._in_dropout = False
        self._last_valid_s = now_s
        self._last_positions = dict(validation.positions)
        self._missing = []
        self._rate.add(now_s)

    def note_dropout(self, reason: str) -> None:
        """
        Registrar una interrupción del enlace detectada por vencimiento del *timeout*.

        Sólo se contabiliza el flanco: mientras el enlace siga caído no se incrementa
        el contador en cada ciclo de diagnóstico.

        :param reason: descripción de la causa observada.
        """
        if not self._in_dropout:
            self._dropouts += 1
            self._in_dropout = True
        self._last_error = reason

    def note_error(self, reason: str) -> None:
        """Registrar el último error observado sin contabilizar una interrupción."""
        self._last_error = reason

    # ------------------------------------------------------------------ consulta ---
    @property
    def received(self) -> int:
        """Total de mensajes recibidos, válidos o no."""
        return self._received

    @property
    def rejected(self) -> int:
        """Total de mensajes rechazados por la validación."""
        return self._rejected

    @property
    def dropouts(self) -> int:
        """Número de interrupciones del enlace detectadas."""
        return self._dropouts

    @property
    def last_error(self) -> str:
        """Último error observado, o cadena vacía si aún no ha ocurrido ninguno."""
        return self._last_error

    @property
    def observation_s(self) -> float:
        """Tiempo de observación continua acumulado en el tramo actual, en segundos."""
        if self._first_valid_s is None or self._last_valid_s is None:
            return 0.0
        return max(0.0, self._last_valid_s - self._first_valid_s)

    def rate_is_reliable(self) -> bool:
        """
        Indicar si la frecuencia estimada ya es representativa.

        :returns: ``True`` cuando se lleva observando el enlace de forma continua al
            menos ``min_span_s`` segundos y hay al menos dos muestras en la ventana.
        """
        return self._rate.samples >= 2 and self.observation_s >= self._min_span_s

    @property
    def positions(self) -> Dict[str, float]:
        """Últimas posiciones válidas conocidas de las articulaciones esperadas."""
        return dict(self._last_positions)

    @property
    def missing(self) -> List[str]:
        """Articulaciones esperadas ausentes en el último mensaje."""
        return list(self._missing)

    @property
    def detected(self) -> List[str]:
        """Articulaciones esperadas detectadas en el último mensaje válido."""
        return [j for j in self.expected_joints if j in self._last_positions]

    def hz(self) -> float:
        """Frecuencia estimada de ``/joint_states``, en Hz."""
        return self._rate.hz()

    def max_gap(self) -> float:
        """Mayor intervalo observado entre mensajes de la ventana, en segundos."""
        return self._rate.max_gap()

    def age(self, now_s: float) -> Optional[float]:
        """
        Edad del último mensaje válido.

        :param now_s: instante actual, en segundos monótonos.
        :returns: edad en segundos, o ``None`` si nunca llegó un mensaje válido.
        """
        if self._last_valid_s is None:
            return None
        return max(0.0, now_s - self._last_valid_s)

    def is_fresh(self, now_s: float) -> bool:
        """Indicar si la telemetría está dentro del *timeout* configurado."""
        age = self.age(now_s)
        return age is not None and age <= self.timeout_s

    def classify(self, now_s: float) -> Tuple[str, str, str]:
        """
        Clasificar el estado del enlace y proponer una acción correctiva.

        :param now_s: instante actual, en segundos monótonos.
        :returns: tupla ``(estado, motivo, acción recomendada)``.
        """
        age = self.age(now_s)
        if age is None:
            return (
                STATE_ERROR,
                'no se ha recibido ningún /joint_states válido',
                'Verifica que el driver esté corriendo (ros2 node list) y que ambas '
                'estaciones compartan ROS_DOMAIN_ID y red.',
            )
        if age > self.timeout_s:
            return (
                STATE_ERROR,
                f'telemetría vencida: {age:.2f} s sin mensaje válido '
                f'(límite {self.timeout_s:.2f} s)',
                'Revisa el enlace Ethernet al robot, el proceso del driver y la '
                'conectividad DDS entre estaciones.',
            )
        if self._missing:
            return (
                STATE_ERROR,
                f"articulaciones faltantes: {', '.join(self._missing)}",
                'Confirma que el bringup se lanzó con el dof correcto (este brazo es de 6 GDL) '
                'y el modelo Gen3 adecuado.',
            )
        hz = self.hz()
        if not self.rate_is_reliable():
            return (
                STATE_WARN,
                f'muestras insuficientes para estimar la frecuencia '
                f'({self._rate.samples} muestras en {self.observation_s:.2f} s; se '
                f'requieren al menos {self._min_span_s:.2f} s de observación continua)',
                'Espera unos segundos a que se llene la ventana de estimación.',
            )
        if hz < self.min_hz:
            return (
                STATE_WARN,
                f'frecuencia degradada: {hz:.1f} Hz por debajo del mínimo '
                f'{self.min_hz:.1f} Hz',
                'Revisa carga de CPU, saturación de la red WiFi y perfiles de QoS.',
            )
        return (
            STATE_OK,
            f'telemetría saludable: {hz:.1f} Hz, edad {age:.3f} s, '
            f'{len(self.detected)}/{len(self.expected_joints)} articulaciones',
            'Ninguna.',
        )
