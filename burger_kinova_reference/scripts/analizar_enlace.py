#!/usr/bin/env python3
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
Análisis de una corrida de ``benchmark_enlace_kinova.sh``.

Responde con datos **no sesgados** a la pregunta de si el enlace sostiene el ciclo de
control del Kinova. Es una distinción importante: ``controller_manager`` sólo imprime
sus tiempos *cuando un ciclo se pasa del presupuesto*, así que promediar esas líneas
mide únicamente los ciclos malos. Aquí se reconstruye la distribución completa a partir
de los tiempos de llegada de **todos** los mensajes de ``/joint_states`` grabados.

Uso::

    python3 scripts/analizar_enlace.py benchmark_wifi_wsl2
    python3 scripts/analizar_enlace.py benchmark_wifi_wsl2 benchmark_ethernet_wsl2
"""

import os
import re
import sys
from typing import Dict, List, Optional

from rclpy.serialization import deserialize_message

import rosbag2_py

from rosidl_runtime_py.utilities import get_message


def percentil(valores: List[float], fraccion: float) -> float:
    """
    Calcular un percentil por interpolación del vecino más cercano.

    :param valores: muestras ya ordenadas.
    :param fraccion: percentil deseado en ``[0, 1]``.
    :returns: el valor del percentil, o ``0.0`` si no hay muestras.
    """
    if not valores:
        return 0.0
    indice = min(len(valores) - 1, max(0, int(round(fraccion * (len(valores) - 1)))))
    return valores[indice]


def _esta_comprimida(ruta: str) -> bool:
    """
    Indicar si la bolsa declara compresión en sus metadatos.

    :param ruta: carpeta de la bolsa.
    :returns: ``True`` si ``metadata.yaml`` declara un modo de compresión distinto de
        ``NONE``, o si sólo hay archivos ``.zstd`` en la carpeta.
    """
    meta = os.path.join(ruta, 'metadata.yaml')
    if os.path.isfile(meta):
        texto = open(meta, encoding='utf-8', errors='ignore').read()
        for linea in texto.splitlines():
            if 'compression_mode' in linea:
                valor = linea.split(':', 1)[1].strip().strip('"\'').upper()
                return valor not in ('', 'NONE')
    return any(f.endswith('.zstd') for f in os.listdir(ruta))


def _uri_de_bolsa(ruta: str) -> str:
    """
    Devolver la URI que rosbag2 puede abrir.

    Una grabación interrumpida con Ctrl+C (o con SIGINT desde un script) puede quedarse
    sin ``metadata.yaml``, y entonces rosbag2 rechaza la carpeta. El archivo ``.mcap``
    sigue siendo legible por sí solo porque MCAP lleva los esquemas embebidos y un
    índice lineal de *chunks*: es justo la propiedad que lo hace resistente a cierres
    abruptos, frente a SQLite3. En ese caso se abre el archivo directamente.

    :param ruta: carpeta de la bolsa.
    :returns: la carpeta si está completa, o la ruta del ``.mcap`` que contiene.
    """
    if os.path.isfile(os.path.join(ruta, 'metadata.yaml')):
        return ruta
    archivos = sorted(
        os.path.join(ruta, f) for f in os.listdir(ruta) if f.endswith('.mcap')
    )
    if not archivos:
        raise FileNotFoundError(f'no hay ni metadata.yaml ni archivos .mcap en {ruta}')
    if len(archivos) > 1:
        print(f'  ⚠ {ruta}: sin metadata.yaml y con {len(archivos)} fragmentos; '
              f'se analiza sólo {os.path.basename(archivos[0])}')
    else:
        print(f'  ⚠ {ruta}: grabación sin metadata.yaml (cierre abrupto); '
              f'se lee {os.path.basename(archivos[0])} directamente')
    return archivos[0]


def leer_bag(ruta: str) -> Dict[str, List]:
    """
    Extraer de la bolsa los tiempos de llegada y los eventos relevantes.

    :param ruta: carpeta de la bolsa MCAP.
    :returns: diccionario con ``joint_stamps``, ``overruns``, ``kortex_timeouts`` y
        ``transiciones``.
    """
    uri = _uri_de_bolsa(ruta)
    # Una bolsa grabada con --compression-mode file guarda un .mcap.zstd, y el lector
    # secuencial normal intentaría interpretarlo como MCAP crudo ("invalid magic bytes").
    # rosbag2 expone un lector aparte que descomprime al vuelo.
    if _esta_comprimida(ruta):
        lector = rosbag2_py.SequentialCompressionReader()
    else:
        lector = rosbag2_py.SequentialReader()
    lector.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                    output_serialization_format='cdr'),
    )
    tipos = {t.name: t.type for t in lector.get_all_topics_and_types()}

    joint_stamps: List[float] = []
    overruns: List[float] = []
    kortex_timeouts: List[float] = []
    transiciones: List[str] = []

    while lector.has_next():
        topico, datos, t_ns = lector.read_next()
        t = t_ns * 1e-9
        if topico == '/joint_states':
            joint_stamps.append(t)
        elif topico == '/rosout':
            msg = deserialize_message(datos, get_message(tipos[topico]))
            texto = msg.msg
            if 'Overrun detected' in texto:
                overruns.append(t)
            elif 'BaseCyclicClient::Refresh' in texto or 'timeout detected' in texto:
                kortex_timeouts.append(t)
            elif 'TRANSICIÓN' in texto:
                transiciones.append(texto.split(']')[-1].strip())

    return {
        'joint_stamps': joint_stamps,
        'overruns': overruns,
        'kortex_timeouts': kortex_timeouts,
        'transiciones': transiciones,
    }


def separar_sesiones(stamps: List[float], corte_s: float = 30.0) -> List[List[float]]:
    """
    Partir los tiempos de llegada en sesiones contiguas de grabación.

    Un hueco enorme entre mensajes no es una caída del enlace: es la firma de una bolsa
    que contiene varias corridas, porque un grabador anterior siguió vivo. Promediar
    sobre eso inventa un ``intervalo máximo`` de minutos y hunde la frecuencia media, así
    que las sesiones se separan y sólo se analiza la primera.

    :param stamps: tiempos de llegada ordenados, en segundos.
    :param corte_s: hueco a partir del cual se considera otra sesión.
    :returns: lista de sesiones, cada una con sus tiempos de llegada.
    """
    if not stamps:
        return []
    sesiones: List[List[float]] = [[stamps[0]]]
    for anterior, actual in zip(stamps, stamps[1:]):
        if actual - anterior > corte_s:
            sesiones.append([])
        sesiones[-1].append(actual)
    return sesiones


def _campo(texto: str, clave: str) -> str:
    """Extraer un campo ``clave : valor`` de ``entorno.txt``."""
    for linea in texto.splitlines():
        if linea.startswith(clave):
            return linea.split(':', 1)[1].strip()
    return '(desconocido)'


def _ping(texto: str) -> List[str]:
    """Extraer las líneas de estadística ``rtt`` de ``ping.txt``."""
    return re.findall(r'rtt min/avg/max/mdev = ([\d./]+) ms', texto)


def analizar(carpeta: str) -> Optional[Dict]:
    """
    Analizar una corrida completa.

    :param carpeta: carpeta ``benchmark_<etiqueta>`` producida por el script.
    :returns: diccionario de métricas, o ``None`` si la corrida está incompleta.
    """
    bag = os.path.join(carpeta, 'bag')
    if not os.path.isdir(bag):
        print(f'  ✗ {carpeta}: no se encontró la bolsa en {bag}')
        return None

    entorno = ''
    ruta_entorno = os.path.join(carpeta, 'entorno.txt')
    if os.path.isfile(ruta_entorno):
        entorno = open(ruta_entorno, encoding='utf-8', errors='ignore').read()

    ping_txt = ''
    ruta_ping = os.path.join(carpeta, 'ping.txt')
    if os.path.isfile(ruta_ping):
        ping_txt = open(ruta_ping, encoding='utf-8', errors='ignore').read()

    datos = leer_bag(bag)
    stamps = sorted(datos['joint_stamps'])
    sesiones = separar_sesiones(stamps)
    if len(sesiones) > 1:
        duraciones = ', '.join(f'{(s[-1] - s[0]):.0f} s' for s in sesiones)
        print(f'  ⚠ {carpeta}: la bolsa contiene {len(sesiones)} sesiones separadas '
              f'({duraciones}).')
        print('    Lo habitual es que un "ros2 bag record" anterior siguiera vivo y '
              'grabara también')
        print('    las corridas siguientes. Se analiza SÓLO la primera sesión; '
              'verifica con')
        print("    pgrep -f 'bag record' que no queden grabadores entre ramas del "
              'experimento.')
    stamps = sesiones[0] if sesiones else []
    intervalos = sorted((b - a) * 1000.0 for a, b in zip(stamps, stamps[1:]))
    duracion = (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0

    return {
        'carpeta': carpeta,
        'sesiones': len(sesiones),
        'etiqueta': _campo(entorno, 'etiqueta'),
        'rmw': _campo(entorno, 'RMW_IMPLEMENTATION'),
        'ping': _ping(ping_txt),
        'mensajes': len(stamps),
        'duracion': duracion,
        'hz': (len(stamps) - 1) / duracion if duracion > 0 else 0.0,
        'intervalos': intervalos,
        'overruns': len(datos['overruns']),
        'kortex_timeouts': len(datos['kortex_timeouts']),
        'transiciones': datos['transiciones'],
    }


def informe(m: Dict) -> None:
    """Imprimir el informe de una corrida."""
    iv = m['intervalos']
    print(f"\n{'=' * 72}")
    print(f"  {m['etiqueta']}   ({m['carpeta']})")
    print(f"{'=' * 72}")
    print(f"  RMW                  : {m['rmw']}")
    for i, rtt in enumerate(m['ping']):
        destino = 'robot  ' if i == 0 else 'gateway'
        print(f'  ping {destino}         : {rtt} ms (min/avg/max/mdev)')
    if m.get('sesiones', 1) > 1:
        print(f"  ⚠ sesiones en la bolsa : {m['sesiones']} (se analiza sólo la primera)")
    print(f"  duración observada   : {m['duracion']:.1f} s")
    print(f"  mensajes /joint_states: {m['mensajes']}")
    print(f"  frecuencia media     : {m['hz']:.2f} Hz")
    if iv:
        print('  --- intervalo entre mensajes (distribución COMPLETA, sin sesgo) ---')
        print(f'    p50  {percentil(iv, 0.50):8.2f} ms')
        print(f'    p90  {percentil(iv, 0.90):8.2f} ms')
        print(f'    p99  {percentil(iv, 0.99):8.2f} ms')
        print(f'    max  {iv[-1]:8.2f} ms')
        for umbral in (20.0, 50.0, 100.0, 1000.0):
            n = sum(1 for v in iv if v > umbral)
            if n:
                print(f'    intervalos > {umbral:7.0f} ms : {n}')
    print(f"  overruns del controller_manager : {m['overruns']}")
    print(f"  timeouts de la API Kortex       : {m['kortex_timeouts']}")
    if m['transiciones']:
        print('  transiciones de estado del enlace:')
        for t in m['transiciones']:
            print(f'    {t}')


def comparar(a: Dict, b: Dict) -> None:
    """Imprimir la comparación entre dos corridas."""
    print(f"\n{'=' * 72}")
    print(f"  COMPARACIÓN: {a['etiqueta']}  vs  {b['etiqueta']}")
    print(f"{'=' * 72}")
    filas = [
        ('frecuencia media (Hz)', a['hz'], b['hz'], '{:.2f}'),
        ('intervalo p50 (ms)', percentil(a['intervalos'], 0.50),
         percentil(b['intervalos'], 0.50), '{:.2f}'),
        ('intervalo p99 (ms)', percentil(a['intervalos'], 0.99),
         percentil(b['intervalos'], 0.99), '{:.2f}'),
        ('intervalo máximo (ms)', a['intervalos'][-1] if a['intervalos'] else 0.0,
         b['intervalos'][-1] if b['intervalos'] else 0.0, '{:.2f}'),
        ('overruns', a['overruns'], b['overruns'], '{:.0f}'),
        ('timeouts Kortex', a['kortex_timeouts'], b['kortex_timeouts'], '{:.0f}'),
    ]
    print(f"  {'métrica':<26}{a['etiqueta']:>18}{b['etiqueta']:>18}")
    for nombre, va, vb, fmt in filas:
        print(f'  {nombre:<26}{fmt.format(va):>18}{fmt.format(vb):>18}')
    print('\n  Los overruns y los timeouts de Kortex deberían tender a cero cuando el')
    print('  enlace sostiene el ciclo de control. Un p99 muy por encima del p50 indica')
    print('  jitter, que es lo que rompe una sesión cíclica en tiempo real.')


def main() -> int:
    """Punto de entrada del analizador."""
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    metricas = [m for m in (analizar(c) for c in sys.argv[1:]) if m]
    for m in metricas:
        informe(m)
    if len(metricas) == 2:
        comparar(metricas[0], metricas[1])
    return 0 if metricas else 1


if __name__ == '__main__':
    sys.exit(main())
