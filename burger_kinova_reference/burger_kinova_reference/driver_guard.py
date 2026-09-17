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
Guarda previa al arranque del driver: no lanzar un segundo driver sobre el mismo robot.

La controladora del Kinova **acepta varias sesiones Kortex a la vez** y no rechaza a un
segundo driver. Lo que sí es único es el *modo de servo* del brazo, y cada driver lo
manipula al arrancar (``SINGLE_LEVEL`` → ``ClearFaults`` → ``LOW_LEVEL``) y al cerrarse
(``SINGLE_LEVEL``). El driver que ya estaba trabajando no se entera: sus comandos empiezan
a fallar con ``WRONG_SERVOING_MODE`` y, mientras ambos envían consignas, el brazo alterna
entre las dos y se mueve a tirones. Ver ``TROUBLESHOOTING.md`` §2.6.

Por eso el launch pregunta **antes** de incluir ``kortex_bringup``. Ninguna vía por sí
sola ve todos los casos, así que se combinan tres, cada una con su alcance:

=====================  ==========================================  ======================
Vía                    Qué detecta                                 Depende del dominio
=====================  ==========================================  ======================
Sesión TCP local       Un driver en **esta** máquina (otra         No
                       terminal, un huérfano)
Grafo DDS              ``/controller_manager`` o publicadores de   Sí (el propio)
                       ``/joint_states`` en cualquier máquina
Anuncio UDP 45455      Un ``kinova_monitor`` que declara ser la    No (misma subred)
                       anfitriona verificada del mismo robot
=====================  ==========================================  ======================

Lo que **no** puede detectar: un driver en otro PC, en otro ``ROS_DOMAIN_ID`` y sin el
monitor del package (por ejemplo ``kortex_bringup`` lanzado a mano). Ese caso sólo se
cierra en el propio robot, cambiando la contraseña de la API Kortex.

Las funciones de evaluación son puras y se prueban sin ROS; sólo
:func:`buscar_driver_activo` toca la red.
"""

import json
import os
import socket
import time
from typing import Dict, Iterable, List, Sequence, Tuple

from burger_kinova_reference.station_announcer import PUERTO_ANUNCIO
from burger_kinova_reference.station_identity import sesiones_locales_hacia

#: Estados de ``/proc/net/tcp`` que implican un driver local vivo.
_TCP_ESTABLECIDA = '01'
_TCP_SYN_SENT = '02'

#: Un anuncio más viejo que esto no se toma como evidencia (el periodo nominal es 5 s).
EDAD_MAXIMA_ANUNCIO_S = 15.0


def evidencia_sesiones_locales(ip_robot: str,
                               sesiones: Iterable[Tuple[int, str]]) -> List[str]:
    """
    Traducir las sesiones TCP locales hacia el robot en evidencia legible.

    :param ip_robot: dirección del robot.
    :param sesiones: pares ``(puerto_remoto, estado_hex)`` de
        :func:`burger_kinova_reference.station_identity.sesiones_locales_hacia`.
    :returns: mensajes de evidencia; vacío si ninguna sesión delata un driver local.
    """
    establecidas = sorted({p for p, estado in sesiones if estado == _TCP_ESTABLECIDA})
    intentando = sorted({p for p, estado in sesiones if estado == _TCP_SYN_SENT})
    evidencias = []
    if establecidas:
        puertos = ','.join(str(p) for p in establecidas)
        evidencias.append(
            f'esta máquina YA tiene una sesión TCP establecida con {ip_robot}:{puertos} '
            f'(otro launch o un driver huérfano; búscalo con "ss -tanp | grep {ip_robot}")')
    if intentando:
        puertos = ','.join(str(p) for p in intentando)
        evidencias.append(
            f'esta máquina ya está intentando conectar con {ip_robot}:{puertos} '
            f'(SYN-SENT): hay otro driver local arrancando')
    return evidencias


def evidencia_grafo_dds(nodos: Iterable[Tuple[str, str]],
                        publicadores_joint_states: int,
                        controller_manager_ns: str = '/controller_manager',
                        joint_state_topic: str = '/joint_states',
                        dominio: str = '0') -> List[str]:
    """
    Buscar en el grafo ROS 2 la huella de un driver ya activo.

    :param nodos: pares ``(nombre, namespace)`` descubiertos.
    :param publicadores_joint_states: publicadores vistos en ``joint_state_topic``.
    :param controller_manager_ns: nombre completo del controller manager.
    :param joint_state_topic: tópico de telemetría articular.
    :param dominio: ``ROS_DOMAIN_ID`` consultado, sólo para el mensaje.
    :returns: mensajes de evidencia; vacío si el grafo no muestra ningún driver.
    """
    objetivo = '/' + controller_manager_ns.strip('/')
    completos = set()
    for nombre, namespace in nodos:
        ns = namespace.rstrip('/')
        completos.add(f'{ns}/{nombre}' if ns else f'/{nombre}')
    evidencias = []
    if objetivo in completos:
        evidencias.append(
            f'ya existe el nodo {objetivo} en ROS_DOMAIN_ID={dominio}: otra estación '
            f'tiene el driver corriendo')
    if publicadores_joint_states > 0:
        evidencias.append(
            f'{joint_state_topic} ya tiene {publicadores_joint_states} publicador(es) en '
            f'ROS_DOMAIN_ID={dominio}: un segundo driver duplicaría la telemetría')
    return evidencias


def evidencia_anuncios(anuncios: Sequence[Tuple[str, Dict]], ip_robot: str,
                       ahora: float,
                       edad_maxima_s: float = EDAD_MAXIMA_ANUNCIO_S) -> List[str]:
    """
    Filtrar los anuncios UDP de estaciones que declaran tener ESTE robot.

    Sólo cuentan los anuncios de rol ``anfitriona`` con ``verificado=si``, dirigidos a la
    misma IP y recientes. Un cliente o una anfitriona de otro robot no bloquean.

    :param anuncios: pares ``(ip_origen, carga)`` recibidos.
    :param ip_robot: dirección del robot que se quiere usar.
    :param ahora: marca de tiempo de referencia.
    :param edad_maxima_s: antigüedad máxima aceptada de un anuncio.
    :returns: un mensaje por estación anfitriona distinta.
    """
    vistas = {}
    for ip_origen, carga in anuncios:
        if not isinstance(carga, dict):
            continue
        if carga.get('rol') != 'anfitriona' or carga.get('verificado') != 'si':
            continue
        if carga.get('robot_ip') != ip_robot:
            continue
        try:
            edad = ahora - float(carga.get('ts', 0.0))
        except (TypeError, ValueError):
            continue
        if edad > edad_maxima_s:
            continue
        estacion = carga.get('estacion') or 'estación sin nombre'
        ip = carga.get('estacion_ip') or ip_origen
        vistas[(estacion, ip)] = carga.get('evidencia', '')
    return [
        f'la estación {estacion} ({ip}) se anuncia como ANFITRIONA de {ip_robot}: '
        f'{evidencia}'
        for (estacion, ip), evidencia in sorted(vistas.items())
    ]


def _abrir_escucha_anuncios(puerto: int):
    """
    Abrir un socket UDP no bloqueante para escuchar los anuncios de estación.

    Se comparte el puerto (``SO_REUSEADDR``/``SO_REUSEPORT``) para no chocar con el
    monitor de red si corre en la misma máquina.

    :param puerto: puerto del anuncio.
    :returns: el socket, o ``None`` si no se pudo abrir.
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, 'SO_REUSEPORT'):
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        sock.bind(('', int(puerto)))
        sock.setblocking(False)
        return sock
    except OSError:
        return None


def _leer_anuncios(sock, destino: List[Tuple[str, Dict]]) -> None:
    """
    Vaciar el socket de anuncios sin bloquear.

    :param sock: socket abierto por :func:`_abrir_escucha_anuncios`.
    :param destino: lista donde se agregan los pares ``(ip_origen, carga)``.
    """
    while True:
        try:
            datos, (ip_origen, _puerto) = sock.recvfrom(65535)
        except (BlockingIOError, InterruptedError):
            return
        except OSError:
            return
        try:
            destino.append((ip_origen, json.loads(datos.decode('utf-8'))))
        except (ValueError, UnicodeDecodeError):
            continue


def buscar_driver_activo(ip_robot: str, use_fake_hardware: bool, timeout_s: float,
                         controller_manager_ns: str = '/controller_manager',
                         joint_state_topic: str = '/joint_states',
                         puerto_anuncio: int = PUERTO_ANUNCIO) -> Tuple[List[str], List[str]]:
    """
    Buscar un driver ya activo antes de lanzar uno nuevo.

    La búsqueda termina en cuanto hay evidencia, o al agotar ``timeout_s``. El tiempo es
    necesario: el descubrimiento DDS entre máquinas tarda del orden de un segundo, y el
    anuncio UDP sale cada 5 s.

    Con hardware simulado no hay sesión Kortex ni anuncio que valga (``robot_ip`` es
    ``0.0.0.0``), así que sólo se consulta el grafo DDS.

    :param ip_robot: dirección del robot.
    :param use_fake_hardware: ``True`` si el driver a lanzar es simulado.
    :param timeout_s: ventana máxima de observación.
    :param controller_manager_ns: nombre completo del controller manager.
    :param joint_state_topic: tópico de telemetría articular.
    :param puerto_anuncio: puerto UDP de los anuncios de estación.
    :returns: tupla ``(evidencias, avisos)``; ``avisos`` describe vías que no se
        pudieron consultar.
    """
    evidencias: List[str] = []
    avisos: List[str] = []

    if not use_fake_hardware:
        evidencias.extend(evidencia_sesiones_locales(
            ip_robot, sesiones_locales_hacia(ip_robot)))
        if evidencias:
            return evidencias, avisos

    dominio = os.environ.get('ROS_DOMAIN_ID', '0') or '0'
    anuncios: List[Tuple[str, Dict]] = []
    sock = None if use_fake_hardware else _abrir_escucha_anuncios(puerto_anuncio)
    if not use_fake_hardware and sock is None:
        avisos.append(
            f'no se pudo escuchar el anuncio UDP {puerto_anuncio}: no se detectarán '
            f'anfitrionas en otros ROS_DOMAIN_ID')

    contexto = nodo = executor = None
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions

        # Contexto propio: no interfiere con el que launch_ros crea después, ni instala
        # manejadores de señal que le roben el Ctrl+C al launch.
        contexto = Context()
        rclpy.init(args=[], context=contexto,
                   signal_handler_options=SignalHandlerOptions.NO)
        nodo = rclpy.create_node(f'burger_driver_guard_{os.getpid()}', context=contexto,
                                 start_parameter_services=False)
        executor = SingleThreadedExecutor(context=contexto)
        executor.add_node(nodo)
    except Exception as exc:  # noqa: BLE001 - la guarda nunca debe tumbar el launch
        avisos.append(f'no se pudo consultar el grafo DDS ({exc})')
        nodo = None

    limite = time.monotonic() + max(0.0, float(timeout_s))
    try:
        while True:
            dds: List[str] = []
            if nodo is not None:
                executor.spin_once(timeout_sec=0.1)
                dds = evidencia_grafo_dds(
                    nodo.get_node_names_and_namespaces(),
                    nodo.count_publishers(joint_state_topic),
                    controller_manager_ns, joint_state_topic, dominio)
            else:
                time.sleep(0.1)
            if sock is not None:
                _leer_anuncios(sock, anuncios)
            udp = evidencia_anuncios(anuncios, ip_robot, time.time())
            if dds or udp:
                # Un instante más para capturar el anuncio que dice QUIÉN es, si el grafo
                # ya delató que existe.
                if dds and not udp and sock is not None and time.monotonic() < limite:
                    fin_extra = min(limite, time.monotonic() + 1.0)
                    while time.monotonic() < fin_extra:
                        time.sleep(0.1)
                        _leer_anuncios(sock, anuncios)
                    udp = evidencia_anuncios(anuncios, ip_robot, time.time())
                evidencias.extend(dds + udp)
                break
            if time.monotonic() >= limite:
                break
    finally:
        if sock is not None:
            sock.close()
        if executor is not None:
            executor.shutdown()
        if nodo is not None:
            nodo.destroy_node()
        if contexto is not None:
            contexto.try_shutdown()

    return evidencias, avisos
