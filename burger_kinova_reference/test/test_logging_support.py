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

"""Pruebas del subsistema de logging: niveles, throttling y transiciones."""

from burger_kinova_reference.logging_support import (
    describe_logging_environment,
    is_valid_severity,
    RECOMMENDED_CONSOLE_FORMAT,
    severity_from_name,
    SEVERITY_POLICY,
    SEVERITY_VALUE,
    StateTransitionLogger,
    ThrottledLogger,
)

from rclpy.logging import LoggingSeverity


class FakeLogger:
    """Logger de prueba que sólo acumula las llamadas recibidas."""

    def __init__(self):
        """Inicializar el registro de llamadas."""
        self.calls = []

    def _record(self, level):
        def _inner(message, **kwargs):
            self.calls.append((level, message, kwargs))
        return _inner

    def __getattr__(self, name):
        """Devolver un método falso para cualquier nivel de severidad."""
        if name in ('debug', 'info', 'warn', 'error', 'fatal'):
            return self._record(name)
        raise AttributeError(name)


def test_niveles_conocidos():
    """Los cinco niveles del taller están mapeados con su valor numérico."""
    assert SEVERITY_VALUE == {'debug': 10, 'info': 20, 'warn': 30, 'error': 40, 'fatal': 50}
    assert set(SEVERITY_POLICY) == set(SEVERITY_VALUE)


def test_conversion_de_nombres():
    """El nombre del nivel se acepta en mayúsculas, minúsculas y con espacios."""
    assert severity_from_name('DEBUG') == LoggingSeverity.DEBUG
    assert severity_from_name(' warn ') == LoggingSeverity.WARN
    assert severity_from_name('warning') == LoggingSeverity.WARN
    assert severity_from_name('inexistente') == LoggingSeverity.INFO
    assert severity_from_name(None) == LoggingSeverity.INFO


def test_validacion_de_nombres():
    """Sólo los nombres conocidos se consideran válidos."""
    assert is_valid_severity('fatal')
    assert not is_valid_severity('trace')
    assert not is_valid_severity(42)


def test_throttling_por_defecto_segun_nivel():
    """DEBUG/WARN/ERROR se limitan por defecto; INFO y FATAL no."""
    fake = FakeLogger()
    throttled = ThrottledLogger(fake, period_s=60.0)
    for _ in range(3):
        throttled.debug('traza', key='t')
        throttled.info('progreso', key='t')
        throttled.warn('degradado', key='t')
        throttled.fatal('crítico', key='t')
    niveles = [level for level, _, _ in fake.calls]
    assert niveles.count('debug') == 1
    assert niveles.count('warn') == 1
    assert niveles.count('info') == 3
    assert niveles.count('fatal') == 3


def test_no_se_pasan_filtros_a_rclpy():
    """
    Ningún mensaje viaja con kwargs de filtro.

    rclpy cachea un contexto por sitio de llamada y prohíbe cambiar severidad, filtros
    o sus parámetros entre llamadas; la envoltura debe llamar siempre igual.
    """
    fake = FakeLogger()
    throttled = ThrottledLogger(fake, period_s=1.0)
    throttled.debug('a')
    throttled.error('b')
    assert all(kwargs == {} for _, _, kwargs in fake.calls)


def test_cada_nivel_tiene_su_propio_cubo():
    """Un WARN limitado no debe suprimir un ERROR emitido desde el mismo sitio."""
    fake = FakeLogger()
    throttled = ThrottledLogger(fake, period_s=60.0)
    throttled.warn('x', key='comun')
    throttled.error('x', key='comun')
    assert [level for level, _, _ in fake.calls] == ['warn', 'error']


def test_sitios_de_llamada_distintos_no_se_interfieren():
    """Sin clave explícita, el cubo es el sitio de llamada real (archivo y línea)."""
    fake = FakeLogger()
    throttled = ThrottledLogger(fake, period_s=60.0)
    throttled.warn('mensaje del sitio A')
    throttled.warn('mensaje del sitio B')
    throttled.warn('mensaje del sitio A otra vez')
    assert len(fake.calls) == 3


def test_mismo_sitio_se_limita():
    """Repetir la misma línea dentro del periodo emite una sola vez."""
    fake = FakeLogger()
    throttled = ThrottledLogger(fake, period_s=60.0)
    for _ in range(5):
        throttled.warn('frecuencia degradada')
    assert len(fake.calls) == 1


def test_throttling_desactivable_por_llamada():
    """Un evento puntual puede saltarse el límite de frecuencia."""
    fake = FakeLogger()
    throttled = ThrottledLogger(fake, period_s=60.0)
    for _ in range(3):
        throttled.error('falla única', throttle=False)
    assert len(fake.calls) == 3


def test_periodo_cero_desactiva_el_throttling():
    """Un periodo de cero segundos equivale a registrar sin límite."""
    fake = FakeLogger()
    throttled = ThrottledLogger(fake, period_s=0.0)
    for _ in range(4):
        throttled.debug('traza')
    assert len(fake.calls) == 4


def test_periodo_ajustable_en_caliente():
    """El periodo puede cambiarse en tiempo de ejecución y nunca es negativo."""
    fake = FakeLogger()
    throttled = ThrottledLogger(fake, period_s=60.0)
    throttled.debug('traza', key='k')
    throttled.debug('traza', key='k')
    assert len(fake.calls) == 1
    throttled.set_period(0.0)
    throttled.debug('traza', key='k')
    assert len(fake.calls) == 2
    throttled.set_period(-5.0)
    assert throttled.period_s == 0.0


def test_solo_registra_transiciones():
    """Repetir el mismo estado no genera líneas nuevas en /rosout."""
    fake = FakeLogger()
    transitions = StateTransitionLogger(ThrottledLogger(fake, 1.0))
    assert transitions.update('OK', 'todo bien')
    assert not transitions.update('OK', 'todo bien')
    assert not transitions.update('OK', 'otro detalle')
    assert transitions.update('ERROR', 'enlace perdido')
    assert len(fake.calls) == 2


def test_severidad_de_la_transicion():
    """Cada transición se emite con la severidad correspondiente al nuevo estado."""
    fake = FakeLogger()
    transitions = StateTransitionLogger(ThrottledLogger(fake, 1.0))
    transitions.update('OK')
    transitions.update('WARN')
    transitions.update('ERROR')
    assert [level for level, _, _ in fake.calls] == ['info', 'warn', 'error']
    assert 'INICIO -> OK' in fake.calls[0][1]
    assert 'WARN -> ERROR' in fake.calls[2][1]


def test_transiciones_pueden_silenciarse():
    """Con log_state_transitions=false no se emite nada, pero el estado se actualiza."""
    fake = FakeLogger()
    transitions = StateTransitionLogger(ThrottledLogger(fake, 1.0), enabled=False)
    assert transitions.update('ERROR')
    assert fake.calls == []
    assert transitions.previous == 'ERROR'


def test_descripcion_del_entorno():
    """El banner reporta las variables que hacen reproducible una captura de consola."""
    env = describe_logging_environment()
    for key in ('RCUTILS_CONSOLE_OUTPUT_FORMAT', 'ROS_LOG_DIR', 'ROS_DOMAIN_ID'):
        assert key in env
    assert '{severity}' in RECOMMENDED_CONSOLE_FORMAT
    assert '{line_number}' in RECOMMENDED_CONSOLE_FORMAT
