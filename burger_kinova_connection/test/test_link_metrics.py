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

"""Pruebas de validación de mensajes, frecuencia y vencimiento del enlace (RF-03)."""

from burger_kinova_connection.link_metrics import (
    LinkHealth,
    RateEstimator,
    STATE_ERROR,
    STATE_OK,
    STATE_WARN,
    validate_joint_state,
    worst_state,
)

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']


def _full_message(offset=0.0):
    """Construir un mensaje completo y coherente de las siete articulaciones."""
    return list(JOINTS), [offset + 0.01 * index for index in range(7)]


def test_valida_mensaje_completo():
    """Un mensaje con las siete articulaciones y arreglos coherentes se acepta."""
    names, positions = _full_message()
    result = validate_joint_state(names, positions, JOINTS)
    assert result.valid
    assert result.missing == []
    assert set(result.positions) == set(JOINTS)


def test_no_depende_del_orden_del_arreglo():
    """La presencia de las articulaciones se comprueba por nombre, no por posición."""
    names = list(reversed(JOINTS))
    positions = [float(index) for index in range(7)]
    result = validate_joint_state(names, positions, JOINTS)
    assert result.valid
    assert result.positions['joint_1'] == 6.0
    assert result.positions['joint_7'] == 0.0


def test_ignora_articulaciones_adicionales_de_la_pinza():
    """Las articulaciones extra del gripper no invalidan el mensaje."""
    names = JOINTS + ['robotiq_85_left_knuckle_joint']
    positions = [0.0] * 8
    assert validate_joint_state(names, positions, JOINTS).valid


def test_rechaza_arreglos_incoherentes():
    """Nombres y posiciones de distinta longitud producen un rechazo explícito."""
    result = validate_joint_state(JOINTS, [0.0, 0.0, 0.0], JOINTS)
    assert not result.valid
    assert 'incoherentes' in result.reason


def test_rechaza_mensaje_sin_nombres():
    """Un mensaje sin nombres no es utilizable."""
    result = validate_joint_state([], [], JOINTS)
    assert not result.valid
    assert result.missing == JOINTS


def test_reporta_articulaciones_faltantes():
    """Un mensaje incompleto lista exactamente qué articulaciones faltan."""
    names = JOINTS[:5]
    result = validate_joint_state(names, [0.0] * 5, JOINTS)
    assert not result.valid
    assert result.missing == ['joint_6', 'joint_7']


def test_rechaza_posiciones_no_finitas():
    """Un NaN en una articulación esperada invalida el mensaje."""
    names, positions = _full_message()
    positions[3] = float('nan')
    result = validate_joint_state(names, positions, JOINTS)
    assert not result.valid
    assert 'joint_4' in result.reason


def test_estimador_de_frecuencia():
    """La frecuencia media se calcula sobre la ventana deslizante."""
    estimator = RateEstimator(window_samples=10)
    for index in range(10):
        estimator.add(index * 0.025)
    assert abs(estimator.hz() - 40.0) < 0.001
    assert abs(estimator.max_gap() - 0.025) < 1e-9


def test_estimador_reporta_el_lapso_de_la_ventana():
    """El lapso cubierto por la ventana se expone para validar la estimación."""
    estimator = RateEstimator(window_samples=10)
    for index in range(5):
        estimator.add(index * 0.1)
    assert abs(estimator.span() - 0.4) < 1e-9


def test_rafaga_de_dds_no_se_toma_como_enlace_saludable():
    """
    Una ráfaga inicial de mensajes encolados no debe leerse como enlace sano.

    Al suscribirse, DDS entrega de golpe lo que tenía en cola: la ventana se llena en
    microsegundos y la frecuencia estimada resulta absurda.
    """
    health = LinkHealth(JOINTS, timeout_s=1.0, min_hz=20.0, min_span_s=0.5)
    _feed(health, 40, period=0.0002)   # 40 mensajes en 8 ms -> ~5000 Hz aparentes
    assert not health.rate_is_reliable()
    state, reason, _ = health.classify(0.01)
    assert state == STATE_WARN
    assert 'insuficientes' in reason


def test_frecuencia_creible_tras_la_ventana_minima():
    """Superado el lapso mínimo de observación, la estimación pasa a ser confiable."""
    health = LinkHealth(JOINTS, timeout_s=1.0, min_hz=20.0, min_span_s=0.5)
    last = _feed(health, 40, period=0.025)   # 40 muestras en ~1 s
    assert health.rate_is_reliable()
    state, _, _ = health.classify(last + 0.01)
    assert state == STATE_OK


def test_ventana_llena_a_alta_frecuencia_sigue_siendo_saludable():
    """
    Una ventana saturada no debe bloquear el estado OK.

    Con 50 muestras a 100 Hz el lapso de la ventana se satura en 0.49 s. Si el umbral
    se midiera sobre la ventana y no sobre el tiempo observado, el enlace quedaría
    marcado como degradado de forma permanente.
    """
    health = LinkHealth(JOINTS, timeout_s=1.0, min_hz=20.0,
                        window_samples=50, min_span_s=0.5)
    last = _feed(health, 300, period=0.01)   # 3 s reales a 100 Hz
    assert health.rate_is_reliable()
    assert health.observation_s > 0.5
    state, reason, _ = health.classify(last + 0.005)
    assert state == STATE_OK, reason


def test_la_observacion_se_reinicia_tras_una_interrupcion():
    """Tras una caída, la frecuencia vuelve a considerarse no confiable hasta reobservar."""
    health = LinkHealth(JOINTS, timeout_s=1.0, min_hz=20.0, min_span_s=0.5)
    last = _feed(health, 200, period=0.01)
    assert health.rate_is_reliable()
    health.note_dropout('enlace perdido')
    _feed(health, 5, start=last + 10.0, period=0.001)
    assert not health.rate_is_reliable()


def test_sin_errores_el_ultimo_error_esta_vacio():
    """Un enlace que nunca falló no debe reportar un error heredado del arranque."""
    health = LinkHealth(JOINTS)
    assert health.last_error == ''
    _feed(health, 30)
    assert health.last_error == ''


def test_estimador_sin_datos_suficientes():
    """Con menos de dos muestras la frecuencia estimada es cero."""
    estimator = RateEstimator()
    assert estimator.hz() == 0.0
    estimator.add(1.0)
    assert estimator.hz() == 0.0


def _feed(health, count, start=0.0, period=0.025):
    """Alimentar el acumulador con mensajes válidos a periodo constante."""
    stamp = start
    for _ in range(count):
        names, positions = _full_message()
        health.update(validate_joint_state(names, positions, JOINTS), stamp)
        stamp += period
    return stamp - period


def test_enlace_saludable():
    """Telemetría fresca, completa y por encima del mínimo se clasifica como OK."""
    health = LinkHealth(JOINTS, timeout_s=1.0, min_hz=20.0)
    last = _feed(health, 30)
    state, reason, _ = health.classify(last + 0.01)
    assert state == STATE_OK
    assert 'saludable' in reason


def test_enlace_degradado_por_frecuencia():
    """Una frecuencia por debajo del mínimo produce WARN, no ERROR."""
    health = LinkHealth(JOINTS, timeout_s=5.0, min_hz=20.0)
    last = _feed(health, 10, period=0.2)  # 5 Hz
    state, reason, _ = health.classify(last + 0.01)
    assert state == STATE_WARN
    assert 'degradada' in reason


def test_enlace_perdido_por_timeout():
    """Superado el timeout sin mensajes el enlace pasa a ERROR."""
    health = LinkHealth(JOINTS, timeout_s=1.0, min_hz=20.0)
    last = _feed(health, 30)
    state, reason, action = health.classify(last + 2.5)
    assert state == STATE_ERROR
    assert 'vencida' in reason
    assert action


def test_sin_telemetria_inicial_es_error():
    """Antes del primer mensaje el estado es ERROR con acción recomendada."""
    health = LinkHealth(JOINTS)
    state, reason, action = health.classify(0.0)
    assert state == STATE_ERROR
    assert 'ningún' in reason
    assert 'ROS_DOMAIN_ID' in action


def test_contabiliza_interrupciones_solo_en_el_flanco():
    """Una caída sostenida cuenta como una única interrupción."""
    health = LinkHealth(JOINTS, timeout_s=1.0)
    last = _feed(health, 30)
    health.note_dropout('sin datos')
    health.note_dropout('sin datos')
    assert health.dropouts == 1
    _feed(health, 5, start=last + 5.0)
    health.note_dropout('sin datos otra vez')
    assert health.dropouts == 2


def test_mensajes_rechazados_se_contabilizan():
    """Los mensajes inválidos incrementan el contador de rechazos."""
    health = LinkHealth(JOINTS)
    health.update(validate_joint_state(JOINTS, [0.0], JOINTS), 0.0)
    assert health.received == 1
    assert health.rejected == 1
    assert 'incoherentes' in health.last_error


def test_worst_state():
    """La combinación de estados devuelve siempre el más grave."""
    assert worst_state(STATE_OK, STATE_WARN) == STATE_WARN
    assert worst_state(STATE_WARN, STATE_ERROR) == STATE_ERROR
    assert worst_state(STATE_OK, STATE_OK) == STATE_OK
    assert worst_state() == STATE_OK
