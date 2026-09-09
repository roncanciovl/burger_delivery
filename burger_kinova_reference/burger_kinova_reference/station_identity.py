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
Identidad de la estación frente al driver del Kinova.

Con un solo robot y varios equipos, el cuello de botella del laboratorio no es que el
robot esté ocupado —lo estará casi siempre— sino **saber qué máquina lo tiene**. Y esa
pregunta no se puede responder por introspección de ROS 2: ``ros2 topic info --verbose``
devuelve el GID del participante DDS, sin hostname ni IP.

La solución que implementa este módulo se apoya en un principio simple:

    **Cada máquina sólo puede afirmar con certeza sobre sí misma, así que que se
    anuncie ella.**

Cada `kinova_monitor` comprueba **localmente** si su propia máquina mantiene la sesión
TCP con la controladora del robot, y publica la respuesta en
``/burger/kinova/diagnostics``. Cualquier estación del mismo dominio lee ahí quién es la
anfitriona, con su hostname y su IP. No hay configuración que mantener, no hay
heurísticas, y si mañana la anfitriona es otro computador, el diagnóstico lo refleja solo.

La comprobación local se hace sobre ``/proc/net/tcp``, que sólo describe **esta** máquina:
en una red conmutada no se pueden ver las conexiones TCP de otro equipo. Esa limitación es
justamente la razón de anunciarse por DDS en vez de intentar detectar al vecino.
"""

import os
import socket
import struct
from typing import Dict, List, Optional, Tuple

#: Estados de ``/proc/net/tcp`` que interesan (columna ``st``, en hexadecimal).
_TCP_ESTABLECIDA = '01'
_TCP_SYN_SENT = '02'

#: Puerto de control de la API Kortex observado en el Gen3. Se usa sólo para informar;
#: la detección no depende de él, porque cualquier sesión hacia la IP del robot cuenta.
PUERTO_KORTEX = 10000

#: Rol de la estación dentro de la arquitectura del proyecto.
ROL_ANFITRIONA = 'anfitriona'
ROL_CLIENTE = 'cliente'
ROL_DESCONOCIDO = 'desconocido'


def _hex_a_ipv4(hex_ip: str) -> str:
    """
    Convertir la IPv4 en hexadecimal *little endian* de ``/proc/net/tcp`` a texto.

    :param hex_ip: dirección tal como aparece en el archivo (8 dígitos hex).
    :returns: la dirección en notación decimal punteada.
    """
    return socket.inet_ntoa(struct.pack('<L', int(hex_ip, 16)))


def _hex_a_ipv6(hex_ip: str) -> str:
    """
    Convertir la IPv6 en hexadecimal de ``/proc/net/tcp6`` a texto.

    :param hex_ip: dirección tal como aparece en el archivo (32 dígitos hex).
    :returns: la dirección en notación IPv6, o cadena vacía si no se puede interpretar.
    """
    try:
        grupos = [hex_ip[i:i + 8] for i in range(0, 32, 8)]
        crudo = b''.join(struct.pack('<L', int(g, 16)) for g in grupos)
        return socket.inet_ntop(socket.AF_INET6, crudo)
    except (ValueError, OSError, struct.error):
        return ''


def sesiones_locales_hacia(ip_robot: str,
                           rutas: Optional[List[str]] = None) -> List[Tuple[int, str]]:
    """
    Listar las sesiones TCP que **esta** máquina mantiene hacia la IP del robot.

    :param ip_robot: dirección del robot.
    :param rutas: archivos a inspeccionar; por defecto ``/proc/net/tcp`` y ``tcp6``.
        Parametrizado para poder probarlo con archivos de ejemplo.
    :returns: lista de ``(puerto_remoto, estado_hex)``; vacía si no hay ninguna.
    """
    if not ip_robot:
        return []
    rutas = rutas if rutas is not None else ['/proc/net/tcp', '/proc/net/tcp6']
    encontradas: List[Tuple[int, str]] = []
    for ruta in rutas:
        try:
            with open(ruta, encoding='utf-8') as fh:
                lineas = fh.readlines()[1:]
        except OSError:
            # Entorno sin /proc (contenedor mínimo, macOS): no es un error, sólo
            # significa que no se puede verificar y así se reportará.
            continue
        for linea in lineas:
            campos = linea.split()
            if len(campos) < 4:
                continue
            try:
                hex_ip, hex_puerto = campos[2].split(':')
            except ValueError:
                continue
            texto = _hex_a_ipv4(hex_ip) if len(hex_ip) == 8 else _hex_a_ipv6(hex_ip)
            if texto.endswith(ip_robot) and (texto == ip_robot or texto.endswith(f':{ip_robot}')):
                try:
                    encontradas.append((int(hex_puerto, 16), campos[3]))
                except ValueError:
                    continue
    return encontradas


def ip_local_hacia(ip_robot: str) -> str:
    """
    Averiguar con qué IP local saldría el tráfico hacia el robot.

    Se usa un socket UDP «conectado»: el sistema resuelve la ruta y asigna la IP de
    origen **sin enviar ni un paquete**, así que no perturba la red ni al robot.

    :param ip_robot: dirección del robot.
    :returns: la IP local de salida, o cadena vacía si no se puede determinar.
    """
    if not ip_robot:
        return ''
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.settimeout(0.2)
        sock.connect((ip_robot, PUERTO_KORTEX))
        return sock.getsockname()[0]
    except OSError:
        return ''
    finally:
        sock.close()


def describir_estacion(ip_robot: str,
                       driver_local: bool,
                       rutas: Optional[List[str]] = None) -> Dict[str, str]:
    """
    Describir el papel de esta estación para publicarlo en el diagnóstico.

    El rol se **verifica** cuando existe una sesión TCP establecida con el robot: eso es
    prueba directa de que este computador es la anfitriona. Si el driver se lanzó aquí
    pero la sesión aún no está establecida, se reporta como declarado y no verificado, en
    lugar de afirmarlo: un diagnóstico que adivina es peor que uno que calla.

    :param ip_robot: dirección del robot configurada.
    :param driver_local: si esta estación lanzó el driver (``start_driver:=true``).
    :param rutas: archivos de ``/proc`` a inspeccionar; para pruebas.
    :returns: diccionario de campos listos para volcar en ``KeyValue``.
    """
    hostname = socket.gethostname()
    sesiones = sesiones_locales_hacia(ip_robot, rutas)
    establecidas = [p for p, estado in sesiones if estado == _TCP_ESTABLECIDA]
    intentando = [p for p, estado in sesiones if estado == _TCP_SYN_SENT]

    if establecidas:
        rol = ROL_ANFITRIONA
        evidencia = (
            f"sesión TCP establecida con {ip_robot}:{','.join(str(p) for p in establecidas)}"
        )
        verificado = 'si'
    elif intentando:
        rol = ROL_DESCONOCIDO
        evidencia = (
            f'intentando conectar con {ip_robot} (SYN-SENT): el robot no responde. '
            f'No es que esté ocupado, es que no es alcanzable'
        )
        verificado = 'no'
    elif driver_local:
        rol = ROL_DESCONOCIDO
        evidencia = (
            'esta estación lanzó el driver pero aún no hay sesión con el robot'
        )
        verificado = 'no'
    else:
        rol = ROL_CLIENTE
        evidencia = f'sin sesión TCP hacia {ip_robot} desde esta máquina'
        verificado = 'si'

    return {
        'estacion': hostname,
        'estacion_ip': ip_local_hacia(ip_robot),
        'estacion_pid': str(os.getpid()),
        'rol_estacion': rol,
        'rol_verificado': verificado,
        'rol_evidencia': evidencia,
    }
