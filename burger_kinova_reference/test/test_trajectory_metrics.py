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

"""Pruebas de las métricas de estabilidad de trayectoria bajo degradación de red."""

import math

from burger_kinova_reference.trajectory_metrics import (
    interval_stats,
    jerk_stats,
    summarize_run,
    tracking_error_stats,
)
import pytest

JOINTS = ['joint_1', 'joint_2']


def test_intervalos_de_una_serie_regular():
    """Una serie a 100 Hz exactos da 100 Hz y 10 ms en todos los percentiles."""
    stamps = [k * 0.01 for k in range(101)]
    stats = interval_stats(stamps)
    assert stats['hz'] == pytest.approx(100.0)
    assert stats['p50_ms'] == pytest.approx(10.0)
    assert stats['max_ms'] == pytest.approx(10.0)


def test_intervalos_detectan_un_hueco():
    """Un hueco de 300 ms aparece en el máximo aunque no mueva la mediana."""
    stamps = [k * 0.01 for k in range(100)] + [0.99 + 0.3]
    stats = interval_stats(stamps)
    assert stats['p50_ms'] == pytest.approx(10.0)
    assert stats['max_ms'] == pytest.approx(300.0)


def test_error_de_seguimiento_por_articulacion():
    """El RMS y el máximo se calculan sobre el valor absoluto, por columna."""
    errors = [[0.01, -0.02], [-0.01, 0.02], [0.01, -0.02]]
    stats = tracking_error_stats(JOINTS, errors)
    assert stats['joint_1']['rms_rad'] == pytest.approx(0.01)
    assert stats['joint_2']['max_rad'] == pytest.approx(0.02)


def test_error_ignora_valores_no_finitos():
    """Un NaN del controlador no contamina el RMS."""
    stats = tracking_error_stats(['joint_1'], [[0.01], [math.nan], [0.01]])
    assert stats['joint_1']['rms_rad'] == pytest.approx(0.01)


def test_jerk_nulo_en_movimiento_uniforme():
    """Velocidad constante: la tercera derivada es cero."""
    stamps = [k * 0.01 for k in range(50)]
    positions = [[0.5 * t, 0.0] for t in stamps]
    stats = jerk_stats(stamps, positions, JOINTS)
    assert stats['joint_1']['max'] == pytest.approx(0.0, abs=1e-6)


def test_jerk_constante_en_movimiento_cubico():
    """Con q = t³ el jerk es constante e igual a 6 rad/s³."""
    stamps = [k * 0.01 for k in range(100)]
    positions = [[t ** 3, 0.0] for t in stamps]
    stats = jerk_stats(stamps, positions, JOINTS)
    assert stats['joint_1']['rms'] == pytest.approx(6.0, rel=1e-3)


def test_jerk_descarta_muestras_casi_simultaneas():
    """Dos mensajes a 1 µs no deben producir un jerk gigantesco."""
    stamps = [k * 0.01 for k in range(20)]
    positions = [[0.5 * t, 0.0] for t in stamps]
    stamps.insert(11, stamps[10] + 1e-6)
    positions.insert(11, [positions[10][0] + 1e-4, 0.0])
    stats = jerk_stats(stamps, positions, JOINTS)
    assert stats['joint_1']['max'] < 1.0


def test_resumen_de_corrida_toma_la_peor_articulacion():
    """El resumen reporta el peor valor entre articulaciones."""
    stamps = [k * 0.01 for k in range(50)]
    positions = [[0.0, 0.0] for _ in stamps]
    errors = [[0.001, 0.05] for _ in stamps]
    row = summarize_run(stamps, positions, JOINTS, errors, JOINTS)
    assert row['error_rms_max_rad'] == pytest.approx(0.05)
    assert row['joint_states_hz'] == pytest.approx(100.0)
