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
Anuncio por broadcast del rol de la estación, para el monitor de red del laboratorio.

Quien tiene el robot es la única máquina capaz de saberlo: el tráfico entre esa estación
y la controladora es **unicast**, y en una red conmutada el switch sólo lo entrega a esos
dos puertos. Ninguna tercera máquina puede observarlo, y el monitor de red del proyecto
además corre sin privilegios —se suscribe al grupo multicast RTPS, no abre sockets raw—,
así que tampoco podría esnifar ARP.

De ahí el diseño: **la estación que tiene la sesión se anuncia**, en vez de intentar
detectarla desde fuera. El anuncio viaja por broadcast UDP, lo que evita configurar la
dirección del monitor en cada estación: el receptor obtiene la IP de origen del propio
``recvfrom`` y la correlaciona con la lista de dispositivos que ya construye.

El contenido no es una inferencia por volumen de tráfico sino **evidencia verificada
localmente** (:mod:`burger_kinova_reference.station_identity` lee ``/proc/net/tcp``).

Limitaciones asumidas, todas documentadas en la interfaz de usuario del monitor:

* El broadcast no cruza subredes. Es la misma condición que ya impone el descubrimiento
  DDS en su rango por defecto, así que no añade ninguna restricción nueva.
* Es un anuncio, no una autoridad: cualquier máquina de la red podría emitir uno falso.
  En una red de laboratorio cerrada resulta aceptable, y el dato se presenta como lo que
  es —lo que esa máquina *declara*—, nunca como prueba irrefutable.
"""

import json
import socket
import time
from typing import Callable, Dict, Optional

#: Puerto UDP del anuncio. Fuera de los rangos que usa DDS para no interferir.
PUERTO_ANUNCIO = 45455

#: Versión del formato, para que el receptor pueda rechazar lo que no entienda.
VERSION_ANUNCIO = 1


def construir_anuncio(identidad: Dict[str, str], robot_ip: str,
                      nodo: str, ahora: Optional[float] = None) -> Dict:
    """
    Construir la carga del anuncio a partir de la identidad ya verificada.

    :param identidad: resultado de
        :func:`burger_kinova_reference.station_identity.describir_estacion`.
    :param robot_ip: dirección del robot a la que se refiere la comprobación.
    :param nodo: nombre del nodo que emite, para poder rastrearlo.
    :param ahora: marca de tiempo; por defecto la del sistema.
    :returns: diccionario serializable a JSON.
    """
    return {
        'v': VERSION_ANUNCIO,
        'estacion': identidad.get('estacion', ''),
        'estacion_ip': identidad.get('estacion_ip', ''),
        'rol': identidad.get('rol_estacion', ''),
        'verificado': identidad.get('rol_verificado', 'no'),
        'evidencia': identidad.get('rol_evidencia', ''),
        'robot_ip': robot_ip,
        'nodo': nodo,
        'ts': time.time() if ahora is None else ahora,
    }


class StationAnnouncer:
    """
    Emisor del anuncio por broadcast UDP.

    Está pensado para no estorbar nunca: si la red no admite broadcast, si el socket
    falla o si el paquete no sale, el nodo sigue funcionando con normalidad. El anuncio
    es una comodidad para el monitor del laboratorio, no una función crítica del enlace
    con el robot.
    """

    def __init__(self, puerto: int = PUERTO_ANUNCIO,
                 destino: str = '255.255.255.255',
                 on_error: Optional[Callable[[str], None]] = None):
        """
        Construir el emisor.

        :param puerto: puerto UDP de destino.
        :param destino: dirección de broadcast.
        :param on_error: callback opcional invocado con el motivo del primer fallo.
        """
        self._puerto = int(puerto)
        self._destino = destino
        self._on_error = on_error
        self._socket: Optional[socket.socket] = None
        self._enviados = 0
        self._fallos = 0
        self._ultimo_error = ''

    @property
    def enviados(self) -> int:
        """Número de anuncios emitidos con éxito."""
        return self._enviados

    @property
    def fallos(self) -> int:
        """Número de anuncios que no se pudieron emitir."""
        return self._fallos

    @property
    def ultimo_error(self) -> str:
        """Motivo del último fallo de emisión, o cadena vacía."""
        return self._ultimo_error

    def _asegurar_socket(self) -> Optional[socket.socket]:
        """Crear el socket de broadcast la primera vez que se necesita."""
        if self._socket is not None:
            return self._socket
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self._socket = sock
            return sock
        except OSError as exc:
            self._registrar_fallo(f'no se pudo crear el socket de broadcast: {exc}')
            return None

    def _registrar_fallo(self, motivo: str) -> None:
        """Contabilizar un fallo y notificarlo la primera vez."""
        self._fallos += 1
        primero = not self._ultimo_error
        self._ultimo_error = motivo
        if primero and self._on_error is not None:
            self._on_error(motivo)

    def anunciar(self, carga: Dict) -> bool:
        """
        Emitir un anuncio.

        :param carga: diccionario serializable, normalmente de :func:`construir_anuncio`.
        :returns: ``True`` si el datagrama salió.
        """
        sock = self._asegurar_socket()
        if sock is None:
            return False
        try:
            sock.sendto(json.dumps(carga).encode('utf-8'),
                        (self._destino, self._puerto))
        except OSError as exc:
            self._registrar_fallo(f'no se pudo emitir el anuncio: {exc}')
            return False
        self._enviados += 1
        return True

    def close(self) -> None:
        """Liberar el socket."""
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None
