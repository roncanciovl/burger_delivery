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

"""Pruebas del búfer circular de la caja negra (patrón Flight Recorder)."""

from burger_kinova_reference.flight_recorder import FlightRecorder


def test_respeta_la_capacidad():
    """El búfer nunca crece por encima de su capacidad."""
    recorder = FlightRecorder(capacity=5)
    for index in range(20):
        recorder.record({'i': index})
    assert len(recorder) == 5
    assert recorder.dropped == 15


def test_conserva_las_muestras_mas_recientes():
    """Al desbordarse se descartan las muestras más antiguas."""
    recorder = FlightRecorder(capacity=3)
    for index in range(6):
        recorder.record({'i': index})
    assert [sample['i'] for sample in recorder.snapshot()] == [3, 4, 5]


def test_snapshot_parcial():
    """Se pueden extraer sólo las últimas N muestras."""
    recorder = FlightRecorder(capacity=10)
    for index in range(10):
        recorder.record({'i': index})
    assert [sample['i'] for sample in recorder.snapshot(last_n=2)] == [8, 9]
    assert recorder.snapshot(last_n=0) == []


def test_deshabilitado_no_registra():
    """Con la caja negra deshabilitada no se consume memoria."""
    recorder = FlightRecorder(capacity=5, enabled=False)
    assert recorder.record({'i': 0}) is False
    assert len(recorder) == 0


def test_capacidad_minima():
    """Una capacidad no positiva se corrige al mínimo utilizable."""
    assert FlightRecorder(capacity=0).capacity == 1


def test_snapshot_es_una_copia():
    """Modificar el snapshot no altera el búfer interno."""
    recorder = FlightRecorder(capacity=3)
    recorder.record({'i': 0})
    snapshot = recorder.snapshot()
    snapshot[0]['i'] = 99
    assert recorder.snapshot()[0]['i'] == 0


def test_anomalia_no_se_limpia_sola():
    """La anomalía permanece activa hasta que un humano la despeja."""
    recorder = FlightRecorder()
    recorder.set_anomaly(True, 'jitter excesivo')
    recorder.record({'i': 0})
    assert recorder.anomaly_active
    assert recorder.anomaly_reason == 'jitter excesivo'
    recorder.set_anomaly(False)
    assert not recorder.anomaly_active
    assert recorder.anomaly_reason == ''


def test_volcado_incluye_encabezado_y_muestras():
    """El volcado numera las muestras y reporta el estado de la anomalía."""
    recorder = FlightRecorder(capacity=4)
    recorder.set_anomaly(True, 'timeout del enlace')
    for index in range(3):
        recorder.record({'t': float(index), 'hz': 40.0})
    lines = recorder.dump_lines()
    assert 'volcado #1' in lines[0]
    assert 'timeout del enlace' in lines[0]
    assert len(lines) == 4
    assert '[0000]' in lines[1]
    assert 'hz=40.00000' in lines[1]


def test_volcado_de_bufer_vacio():
    """Un volcado sin telemetría lo indica explícitamente en lugar de fallar."""
    lines = FlightRecorder().dump_lines()
    assert 'búfer vacío' in lines[1]


def test_contador_de_volcados():
    """Cada volcado incrementa el contador histórico."""
    recorder = FlightRecorder()
    recorder.dump_lines()
    recorder.dump_lines()
    assert recorder.dumps == 2


def test_formato_de_listas_en_el_volcado():
    """Las posiciones articulares se formatean de manera compacta y estable."""
    recorder = FlightRecorder()
    recorder.record({'posiciones': [0.1, -0.2]})
    assert '[0.10000,-0.20000]' in recorder.dump_lines()[1]
