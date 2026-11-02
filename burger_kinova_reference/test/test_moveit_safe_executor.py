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

"""Pruebas de la cancelación por Ctrl+C en ``moveit_safe_executor`` (guía Lab 04, §6.6)."""

import threading
from types import SimpleNamespace

from burger_kinova_reference.moveit_safe_executor import MoveItSafeExecutor

import pytest

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


class _Futuro:
    """Futuro ya resuelto, con la interfaz que usa el ejecutor."""

    def __init__(self, valor=None):
        self._valor = valor

    def done(self):
        return True

    def result(self):
        return self._valor


class _Meta:
    """Meta aceptada que registra si alguien pidió cancelarla."""

    accepted = True

    def __init__(self):
        self.canceladas = 0

    def cancel_goal_async(self):
        self.canceladas += 1
        return _Futuro()


class _Logger:

    def __init__(self):
        self.avisos = []

    def warn(self, texto):
        self.avisos.append(texto)

    error = info = warn


def _ejecutor(meta):
    """Construir el nodo sin ROS: sólo los atributos que usan ``_enviar`` y la parada."""
    nodo = object.__new__(MoveItSafeExecutor)
    logger = _Logger()
    nodo._joints = list(JOINTS)
    nodo._lock = threading.Lock()
    nodo._stop = threading.Event()
    nodo._active_goal = None
    nodo.get_logger = lambda: logger
    nodo.get_parameter = lambda nombre: SimpleNamespace(value=1.0)
    nodo._trajectory_client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda goal, feedback_callback=None: _Futuro(meta))
    return nodo, logger


def _interrumpir(*_args):
    raise KeyboardInterrupt


def test_ctrl_c_durante_el_movimiento_cancela_la_meta():
    """Ctrl+C mientras el brazo se mueve debe pedir la cancelación al controlador."""
    meta = _Meta()
    nodo, logger = _ejecutor(meta)
    nodo._seguir = _interrumpir
    puntos = [(0.0, [0.0] * 6, None), (1.0, [0.1] * 6, None)]

    with pytest.raises(KeyboardInterrupt):
        nodo._enviar('home', puntos, 1.0)

    assert meta.canceladas == 1
    assert nodo._stop.is_set()
    assert nodo._active_goal is None
    assert any('Cancelando la trayectoria' in aviso for aviso in logger.avisos)


def test_parada_sin_meta_en_curso_no_cancela_nada():
    """Tras terminar la meta, ``solicitar_parada`` sólo impide enviar más."""
    meta = _Meta()
    nodo, _ = _ejecutor(meta)

    nodo.solicitar_parada()

    assert meta.canceladas == 0
    assert nodo._stop.is_set()
