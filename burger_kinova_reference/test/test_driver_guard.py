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

"""Pruebas de la guarda que impide lanzar un segundo driver sobre el mismo robot."""

from burger_kinova_reference.driver_guard import (
    evidencia_anuncios,
    evidencia_grafo_dds,
    evidencia_sesiones_locales,
)

IP = '192.168.1.10'


def _anuncio(rol='anfitriona', verificado='si', robot_ip=IP, ts=100.0):
    return {
        'v': 1, 'estacion': 'PC-LAB-02', 'estacion_ip': '192.168.1.88', 'rol': rol,
        'verificado': verificado, 'evidencia': 'sesión TCP establecida con '
        '192.168.1.10:10000', 'robot_ip': robot_ip, 'nodo': 'kinova_monitor', 'ts': ts,
    }


def test_sin_sesiones_locales_no_hay_evidencia():
    """Una máquina limpia no bloquea el arranque."""
    assert evidencia_sesiones_locales(IP, []) == []


def test_sesion_establecida_bloquea():
    """Un driver local (otra terminal o huérfano) se detecta por su sesión TCP."""
    evidencias = evidencia_sesiones_locales(IP, [(10000, '01')])
    assert len(evidencias) == 1
    assert '10000' in evidencias[0]


def test_sesion_en_syn_sent_bloquea():
    """Un driver local arrancando también cuenta."""
    assert evidencia_sesiones_locales(IP, [(10000, '02')])


def test_sesion_cerrandose_no_bloquea():
    """TIME-WAIT (0x06) es una sesión que ya se liberó."""
    assert evidencia_sesiones_locales(IP, [(10000, '06')]) == []


def test_grafo_vacio_no_bloquea():
    """Sin controller manager ni telemetría el camino está libre."""
    assert evidencia_grafo_dds([('kinova_monitor_eq03', '/')], 0) == []


def test_controller_manager_en_el_dominio_bloquea():
    """El nodo /controller_manager delata un driver en otra estación del dominio."""
    evidencias = evidencia_grafo_dds(
        [('controller_manager', '/'), ('robot_state_publisher', '/')], 0, dominio='0')
    assert len(evidencias) == 1
    assert '/controller_manager' in evidencias[0]
    assert 'ROS_DOMAIN_ID=0' in evidencias[0]


def test_controller_manager_con_otro_nombre_no_confunde():
    """Un nodo que sólo contiene el nombre no es el controller manager."""
    assert evidencia_grafo_dds([('controller_manager_spy', '/')], 0) == []


def test_controller_manager_en_namespace():
    """El nombre completo se compara con namespace incluido."""
    nodos = [('controller_manager', '/kinova')]
    assert evidencia_grafo_dds(nodos, 0, controller_manager_ns='/kinova/controller_manager')
    assert evidencia_grafo_dds(nodos, 0) == []


def test_publicadores_de_joint_states_bloquean():
    """Telemetría ya publicada implica un driver activo."""
    evidencias = evidencia_grafo_dds([], 1)
    assert len(evidencias) == 1
    assert '/joint_states' in evidencias[0]


def test_anuncio_de_anfitriona_verificada_bloquea():
    """Una anfitriona del mismo robot, reciente y verificada, se reporta por nombre e IP."""
    evidencias = evidencia_anuncios([('192.168.1.88', _anuncio())], IP, ahora=102.0)
    assert len(evidencias) == 1
    assert 'PC-LAB-02' in evidencias[0]
    assert '192.168.1.88' in evidencias[0]


def test_anuncios_que_no_bloquean():
    """Clientes, roles no verificados, otro robot o anuncios viejos no cuentan."""
    anuncios = [
        ('192.168.1.50', _anuncio(rol='cliente')),
        ('192.168.1.51', _anuncio(verificado='no')),
        ('192.168.1.52', _anuncio(robot_ip='192.168.1.11')),
        ('192.168.1.53', _anuncio(ts=0.0)),
        ('192.168.1.54', 'no es un diccionario'),
        ('192.168.1.55', _anuncio(ts='basura')),
    ]
    assert evidencia_anuncios(anuncios, IP, ahora=100.0) == []


def test_anuncios_repetidos_se_reportan_una_vez():
    """La misma anfitriona anunciándose varias veces genera una sola evidencia."""
    anuncios = [('192.168.1.88', _anuncio(ts=98.0)), ('192.168.1.88', _anuncio(ts=100.0))]
    assert len(evidencia_anuncios(anuncios, IP, ahora=101.0)) == 1
