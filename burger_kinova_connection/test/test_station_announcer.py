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

"""Pruebas del anuncio por broadcast del rol de la estación."""

import json
import socket
import threading

from burger_kinova_connection.station_announcer import (
    construir_anuncio,
    PUERTO_ANUNCIO,
    StationAnnouncer,
    VERSION_ANUNCIO,
)

IDENTIDAD = {
    'estacion': 'PC-LAB-01',
    'estacion_ip': '192.168.1.42',
    'estacion_pid': '4242',
    'rol_estacion': 'anfitriona',
    'rol_verificado': 'si',
    'rol_evidencia': 'sesión TCP establecida con 192.168.1.10:10000',
}

NODO = 'monitor_de_prueba'


def test_anuncio_lleva_los_campos_del_receptor():
    """La carga trae lo que el monitor de red necesita para pintar el badge."""
    carga = construir_anuncio(IDENTIDAD, '192.168.1.10', NODO, ahora=1.0)
    assert carga['v'] == VERSION_ANUNCIO
    assert carga['estacion'] == 'PC-LAB-01'
    assert carga['rol'] == 'anfitriona'
    assert carga['verificado'] == 'si'
    assert carga['robot_ip'] == '192.168.1.10'
    assert carga['nodo'] == NODO
    assert carga['ts'] == 1.0


def test_anuncio_es_serializable():
    """Debe poder viajar como JSON sin tipos exóticos."""
    carga = construir_anuncio(IDENTIDAD, '192.168.1.10', NODO)
    assert json.loads(json.dumps(carga)) == carga


def test_anuncio_tolera_identidad_incompleta():
    """Una identidad parcial no debe romper el anuncio."""
    carga = construir_anuncio({}, '', NODO)
    assert carga['estacion'] == ''
    assert carga['rol'] == ''
    assert json.dumps(carga)


def test_emision_y_recepcion():
    """El datagrama emitido llega íntegro a un receptor local."""
    receptor = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receptor.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    receptor.bind(('', PUERTO_ANUNCIO))
    receptor.settimeout(5)
    recibido = {}

    def escuchar():
        try:
            datos, _ = receptor.recvfrom(4096)
            recibido.update(json.loads(datos.decode()))
        except (socket.timeout, ValueError):
            pass

    hilo = threading.Thread(target=escuchar, daemon=True)
    hilo.start()

    emisor = StationAnnouncer()
    assert emisor.anunciar(construir_anuncio(IDENTIDAD, '192.168.1.10', NODO))
    hilo.join(timeout=5)
    emisor.close()
    receptor.close()

    assert recibido.get('estacion') == 'PC-LAB-01'
    assert recibido.get('rol') == 'anfitriona'
    assert emisor.enviados == 1
    assert emisor.fallos == 0


def test_un_fallo_no_derriba_el_nodo():
    """
    Si la red no admite el broadcast, el nodo continúa.

    El anuncio es una comodidad para el monitor del laboratorio, no una función crítica
    del enlace con el robot.
    """
    motivos = []
    emisor = StationAnnouncer(destino='no-es-una-direccion', on_error=motivos.append)
    assert emisor.anunciar({'v': 1}) is False
    assert emisor.fallos == 1
    assert emisor.enviados == 0
    assert emisor.ultimo_error
    assert len(motivos) == 1
    emisor.close()


def test_el_error_solo_se_notifica_una_vez():
    """Una red sin broadcast no debe inundar /rosout con el mismo aviso."""
    motivos = []
    emisor = StationAnnouncer(destino='no-es-una-direccion', on_error=motivos.append)
    for _ in range(5):
        emisor.anunciar({'v': 1})
    assert emisor.fallos == 5
    assert len(motivos) == 1
    emisor.close()


def test_close_es_idempotente():
    """Cerrar dos veces no debe fallar."""
    emisor = StationAnnouncer()
    emisor.close()
    emisor.close()
