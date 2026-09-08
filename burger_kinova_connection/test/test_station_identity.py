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
Pruebas de la identificación de la estación anfitriona del driver.

Se ejercita contra archivos ``/proc/net/tcp`` sintéticos, de modo que las pruebas no
dependen de la red del laboratorio ni de que el robot esté conectado.
"""

import socket
import struct

from burger_kinova_connection.station_identity import (
    describir_estacion,
    ip_local_hacia,
    ROL_ANFITRIONA,
    ROL_CLIENTE,
    ROL_DESCONOCIDO,
    sesiones_locales_hacia,
)

CABECERA = ('  sl  local_address rem_address   st tx_queue rx_queue tr tm->when '
            'retrnsmt   uid  timeout inode\n')


def _hex_ipv4(ip: str) -> str:
    """Codificar una IPv4 como la escribe el kernel en /proc/net/tcp."""
    return f'{struct.unpack("<L", socket.inet_aton(ip))[0]:08X}'


def _linea(ip_remota: str, puerto: int, estado: str, indice: int = 0) -> str:
    """Construir una línea de /proc/net/tcp hacia ``ip_remota``."""
    local = f'{_hex_ipv4("192.168.1.42")}:C000'
    remoto = f'{_hex_ipv4(ip_remota)}:{puerto:04X}'
    return (f'   {indice}: {local} {remoto} {estado} 00000000:00000000 '
            f'00:00000000 00000000  1001        0 12345 1 0000 100 0 0 10 0\n')


def _archivo(tmp_path, *lineas) -> str:
    """Escribir un /proc/net/tcp sintético y devolver su ruta."""
    ruta = tmp_path / 'tcp'
    ruta.write_text(CABECERA + ''.join(lineas), encoding='utf-8')
    return str(ruta)


def test_detecta_sesion_establecida(tmp_path):
    """Una sesión ESTABLECIDA hacia el robot se localiza con su puerto."""
    ruta = _archivo(tmp_path, _linea('192.168.1.10', 10000, '01'))
    assert sesiones_locales_hacia('192.168.1.10', [ruta]) == [(10000, '01')]


def test_ignora_sesiones_hacia_otras_ips(tmp_path):
    """Las conexiones a otros equipos no se confunden con la del robot."""
    ruta = _archivo(tmp_path,
                    _linea('192.168.1.77', 22, '01', 0),
                    _linea('8.8.8.8', 443, '01', 1))
    assert sesiones_locales_hacia('192.168.1.10', [ruta]) == []


def test_archivo_inexistente_no_revienta():
    """Un entorno sin /proc devuelve lista vacía en lugar de fallar."""
    assert sesiones_locales_hacia('192.168.1.10', ['/no/existe/tcp']) == []


def test_ip_vacia_no_consulta_nada():
    """Sin IP de robot configurada no hay nada que buscar."""
    assert sesiones_locales_hacia('', None) == []


def test_anfitriona_verificada(tmp_path):
    """Con la sesión establecida, la estación se declara anfitriona y verificada."""
    ruta = _archivo(tmp_path, _linea('192.168.1.10', 10000, '01'))
    info = describir_estacion('192.168.1.10', driver_local=True, rutas=[ruta])
    assert info['rol_estacion'] == ROL_ANFITRIONA
    assert info['rol_verificado'] == 'si'
    assert '10000' in info['rol_evidencia']
    assert info['estacion']


def test_anfitriona_se_verifica_aunque_no_declare_el_driver(tmp_path):
    """
    La evidencia manda sobre la declaración.

    Si hay sesión con el robot, esta máquina es la anfitriona aunque el driver lo haya
    lanzado otro proceso (por ejemplo, un huérfano de una corrida anterior).
    """
    ruta = _archivo(tmp_path, _linea('192.168.1.10', 10000, '01'))
    info = describir_estacion('192.168.1.10', driver_local=False, rutas=[ruta])
    assert info['rol_estacion'] == ROL_ANFITRIONA


def test_cliente_sin_sesion(tmp_path):
    """Sin sesión y sin driver local, la estación es cliente, y eso es lo normal."""
    ruta = _archivo(tmp_path, _linea('192.168.1.77', 22, '01'))
    info = describir_estacion('192.168.1.10', driver_local=False, rutas=[ruta])
    assert info['rol_estacion'] == ROL_CLIENTE
    assert info['rol_verificado'] == 'si'


def test_syn_sent_es_robot_inalcanzable(tmp_path):
    """
    SYN-SENT no significa "ocupado" sino "no responde".

    Es la distinción que evita diagnosticar mal: el driver está intentando conectar y el
    robot no contesta.
    """
    ruta = _archivo(tmp_path, _linea('192.168.1.10', 10000, '02'))
    info = describir_estacion('192.168.1.10', driver_local=True, rutas=[ruta])
    assert info['rol_estacion'] == ROL_DESCONOCIDO
    assert info['rol_verificado'] == 'no'
    assert 'no es alcanzable' in info['rol_evidencia']


def test_driver_declarado_sin_sesion_no_se_afirma(tmp_path):
    """Declarar el driver no basta: sin evidencia, el rol queda sin verificar."""
    ruta = _archivo(tmp_path)
    info = describir_estacion('192.168.1.10', driver_local=True, rutas=[ruta])
    assert info['rol_estacion'] == ROL_DESCONOCIDO
    assert info['rol_verificado'] == 'no'


def test_campos_publicables(tmp_path):
    """El diagnóstico recibe siempre el juego completo de campos, como cadenas."""
    ruta = _archivo(tmp_path)
    info = describir_estacion('192.168.1.10', driver_local=False, rutas=[ruta])
    esperados = {'estacion', 'estacion_ip', 'estacion_pid', 'rol_estacion',
                 'rol_verificado', 'rol_evidencia'}
    assert esperados == set(info)
    assert all(isinstance(v, str) for v in info.values())


def test_ip_local_no_envia_trafico():
    """Determinar la IP de salida no debe fallar ni bloquear con una IP inalcanzable."""
    resultado = ip_local_hacia('192.0.2.1')   # TEST-NET-1, no enrutable
    assert isinstance(resultado, str)
