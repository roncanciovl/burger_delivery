#!/usr/bin/env python3
# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""
Receptor de los anuncios de rol de las estaciones ROS 2 del laboratorio.

Responde a la pregunta que más tiempo hace perder cuando hay un solo robot: **¿qué
máquina tiene ocupado el Kinova?**

Esa pregunta no se puede contestar observando la red. El tráfico entre la estación que
ejecuta el driver y la controladora del robot es unicast, y en una red conmutada el
switch sólo lo entrega a esos dos puertos: ninguna tercera máquina lo ve. Sniffear ARP
tampoco sirve —exigiría privilegios de root, que este monitor evita a propósito, y Linux
refresca la entrada ARP con sondas unicast, así que sólo sería visible el instante inicial
en que arranca el driver—.

Por eso la estación que tiene la sesión **se anuncia**: `kinova_monitor` comprueba en su
propio `/proc/net/tcp` si mantiene la sesión TCP con el robot y difunde el resultado por
broadcast UDP. Este módulo lo recibe y lo indexa por IP de origen, que se obtiene del
propio `recvfrom` y encaja directamente con la lista de dispositivos del escáner.

No requiere privilegios ni configuración: sólo escuchar un puerto UDP.

Advertencia deliberada: un anuncio es lo que una máquina **declara**, no una prueba. En
una red de laboratorio cerrada es suficiente; la interfaz lo presenta como tal.
"""

import json
import socket
import threading
import time
from typing import Any, Dict, List, Optional

PUERTO_ANUNCIO = 45455

#: Sin noticias durante este tiempo, la estación se considera ausente. Cubre varios
#: periodos de anuncio para tolerar un datagrama perdido sin parpadear en la interfaz.
TTL_SEGUNDOS = 20.0

#: Tamaño máximo aceptado, para no procesar datagramas absurdos.
MAX_DATAGRAMA = 4096

ROL_ANFITRIONA = 'anfitriona'


class StationListener:
    """Escucha los anuncios y mantiene el último estado conocido de cada estación."""

    def __init__(self, puerto: int = PUERTO_ANUNCIO, ttl: float = TTL_SEGUNDOS):
        """
        Construir el receptor.

        :param puerto: puerto UDP donde se escuchan los anuncios.
        :param ttl: segundos sin recibir tras los que una estación se da por ausente.
        """
        self._puerto = int(puerto)
        self._ttl = float(ttl)
        self._estaciones: Dict[str, Dict[str, Any]] = {}
        self._lock = threading.Lock()
        self._hilo: Optional[threading.Thread] = None
        self._parar = threading.Event()
        self._error: str = ''
        self._recibidos = 0

    @property
    def error(self) -> str:
        """Motivo por el que el receptor no está funcionando, si aplica."""
        return self._error

    def start(self) -> None:
        """Arrancar el hilo de escucha. No lanza si el puerto no se puede abrir."""
        if self._hilo is not None:
            return
        self._hilo = threading.Thread(target=self._bucle, daemon=True,
                                      name='station-listener')
        self._hilo.start()

    def stop(self) -> None:
        """Detener el hilo de escucha."""
        self._parar.set()

    def _bucle(self) -> None:
        """Recibir anuncios hasta que se pida parar."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('', self._puerto))
            sock.settimeout(1.0)
        except OSError as exc:
            self._error = f'no se pudo escuchar en el puerto {self._puerto}: {exc}'
            return

        while not self._parar.is_set():
            try:
                datos, origen = sock.recvfrom(MAX_DATAGRAMA)
            except socket.timeout:
                continue
            except OSError as exc:
                self._error = f'fallo al recibir: {exc}'
                break
            self._procesar(datos, origen[0])
        sock.close()

    def _procesar(self, datos: bytes, ip_origen: str) -> None:
        """
        Validar e indexar un anuncio recibido.

        :param datos: carga del datagrama.
        :param ip_origen: IP del emisor, tomada del socket y no del contenido.
        """
        try:
            carga = json.loads(datos.decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return
        if not isinstance(carga, dict) or carga.get('v') != 1:
            return

        self._recibidos += 1
        with self._lock:
            self._estaciones[ip_origen] = {
                # La IP de origen la fija el socket, no el emisor: es el único campo
                # que un anuncio no puede falsear sin suplantar la dirección.
                'ip': ip_origen,
                'estacion': str(carga.get('estacion', ''))[:64],
                'rol': str(carga.get('rol', ''))[:32],
                'verificado': str(carga.get('verificado', 'no'))[:8],
                'evidencia': str(carga.get('evidencia', ''))[:256],
                'robot_ip': str(carga.get('robot_ip', ''))[:64],
                'nodo': str(carga.get('nodo', ''))[:64],
                'visto': time.time(),
            }

    def _vigentes(self) -> Dict[str, Dict[str, Any]]:
        """Devolver las estaciones cuyo anuncio no ha caducado."""
        limite = time.time() - self._ttl
        with self._lock:
            return {ip: dict(datos) for ip, datos in self._estaciones.items()
                    if datos['visto'] >= limite}

    @staticmethod
    def _con_edad(datos: Dict[str, Any], ahora: float) -> Dict[str, Any]:
        """Añadir la antigüedad del anuncio, en segundos."""
        datos['edad_s'] = round(ahora - datos['visto'], 1)
        return datos

    def estaciones(self) -> List[Dict[str, Any]]:
        """
        Listar las estaciones que están anunciándose ahora mismo.

        :returns: lista de anuncios vigentes, con la antigüedad en segundos.
        """
        ahora = time.time()
        vigentes = [self._con_edad(d, ahora) for d in self._vigentes().values()]
        return sorted(vigentes, key=lambda d: d['ip'])

    def rol_de(self, ip: str) -> Optional[Dict[str, Any]]:
        """
        Consultar el rol anunciado por una IP concreta.

        Devuelve la misma forma que las entradas de :meth:`estaciones`, ``edad_s``
        incluida, para que quien consuma una u otra no tenga que distinguirlas.

        :param ip: dirección del dispositivo.
        :returns: el anuncio vigente, o ``None`` si esa IP no se anuncia.
        """
        datos = self._vigentes().get(ip)
        return None if datos is None else self._con_edad(datos, time.time())

    def anfitriona(self) -> Optional[Dict[str, Any]]:
        """
        Devolver la estación que se declara anfitriona del driver, si alguna.

        Sólo se considera anfitriona la que además marca el rol como verificado: el nodo
        únicamente lo hace cuando encuentra la sesión TCP establecida con el robot.

        :returns: el anuncio de la anfitriona, o ``None``.
        """
        ahora = time.time()
        for datos in self._vigentes().values():
            if datos['rol'] == ROL_ANFITRIONA and datos['verificado'] == 'si':
                return self._con_edad(datos, ahora)
        return None

    def resumen(self) -> Dict[str, Any]:
        """
        Resumir el estado para exponerlo por la API.

        :returns: diccionario con la anfitriona, las estaciones vistas y el estado.
        """
        anfitriona = self.anfitriona()
        return {
            'listener_activo': self._error == '',
            'listener_error': self._error,
            'anuncios_recibidos': self._recibidos,
            'ttl_s': self._ttl,
            'anfitriona': anfitriona,
            'estaciones': self.estaciones(),
        }


if __name__ == '__main__':
    oyente = StationListener()
    oyente.start()
    print(f'Escuchando anuncios de estación en el puerto {PUERTO_ANUNCIO}...')
    try:
        while True:
            time.sleep(3)
            resumen = oyente.resumen()
            if resumen['listener_error']:
                print(f"  error: {resumen['listener_error']}")
                break
            anfitriona = resumen['anfitriona']
            if anfitriona:
                print(f"  ANFITRIONA: {anfitriona['estacion']} ({anfitriona['ip']}) "
                      f"— {anfitriona['evidencia']}")
            else:
                print(f"  sin anfitriona anunciada "
                      f"({len(resumen['estaciones'])} estaciones vistas)")
    except KeyboardInterrupt:
        oyente.stop()
