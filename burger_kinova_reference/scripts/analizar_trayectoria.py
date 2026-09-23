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

r"""
Comparar corridas de ``benchmark_trayectoria_netem.sh`` (una por escenario E1/E2/E3).

Por cada carpeta ``trayectoria_<etiqueta>`` lee la bolsa, calcula con
``burger_kinova_reference.trajectory_metrics`` la cadencia de ``/joint_states``, el error
de seguimiento del ``joint_trajectory_controller`` y el *jerk* medido, y cuenta los
eventos que delatan un enlace malo en ``/rosout`` (overruns, timeouts de Kortex, metas
abortadas). Imprime una tabla y, con ``--csv``, la guarda para el informe.

Uso::

    python3 scripts/analizar_trayectoria.py trayectoria_cliente_E1 trayectoria_cliente_E2 \\
        trayectoria_cliente_E3 --csv resultados_netem.csv
"""

import argparse
import csv
import os
import re
import sys
from typing import Dict, List

from burger_kinova_reference.trajectory_metrics import summarize_run

from rclpy.serialization import deserialize_message

import rosbag2_py

from rosidl_runtime_py.utilities import get_message

EVENTOS = {
    'overruns': re.compile(r'Overrun detected'),
    'timeouts_kortex': re.compile(r'BaseCyclicClient::Refresh|timeout detected'),
    'metas_abortadas': re.compile(r'ABORTED|PATH_TOLERANCE_VIOLATED|GOAL_TOLERANCE_VIOLATED'),
}
ESTADO_CONTROLADOR = (
    '/joint_trajectory_controller/controller_state',
    '/joint_trajectory_controller/state',
)


def _abrir(ruta: str):
    uri = ruta
    if not os.path.isfile(os.path.join(ruta, 'metadata.yaml')):
        mcaps = sorted(f for f in os.listdir(ruta) if f.endswith('.mcap'))
        if not mcaps:
            raise FileNotFoundError(f'{ruta}: ni metadata.yaml ni .mcap')
        uri = os.path.join(ruta, mcaps[0])
    lector = rosbag2_py.SequentialReader()
    lector.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                            output_serialization_format='cdr'))
    return lector


def leer_corrida(carpeta: str) -> Dict:
    """Extraer de la bolsa de una corrida las series que necesitan las métricas."""
    lector = _abrir(os.path.join(carpeta, 'bag'))
    tipos = {t.name: t.type for t in lector.get_all_topics_and_types()}
    js_t: List[float] = []
    js_q: List[List[float]] = []
    js_names: List[str] = []
    err_rows: List[List[float]] = []
    err_names: List[str] = []
    eventos = {clave: 0 for clave in EVENTOS}
    while lector.has_next():
        topico, datos, t_ns = lector.read_next()
        if topico == '/joint_states':
            msg = deserialize_message(datos, get_message(tipos[topico]))
            if not js_names:
                js_names = sorted(n for n in msg.name if re.fullmatch(r'joint_\d+', n))
            indice = {n: i for i, n in enumerate(msg.name)}
            if all(n in indice for n in js_names):
                js_t.append(t_ns * 1e-9)
                js_q.append([msg.position[indice[n]] for n in js_names])
        elif topico in ESTADO_CONTROLADOR:
            msg = deserialize_message(datos, get_message(tipos[topico]))
            err_names = list(msg.joint_names)
            err_rows.append(list(msg.error.positions))
        elif topico == '/rosout':
            texto = deserialize_message(datos, get_message(tipos[topico])).msg
            for clave, patron in EVENTOS.items():
                if patron.search(texto):
                    eventos[clave] += 1
    return {'js_t': js_t, 'js_q': js_q, 'js_names': js_names,
            'err_rows': err_rows, 'err_names': err_names, 'eventos': eventos}


def _campo(carpeta: str, archivo: str, clave: str) -> str:
    ruta = os.path.join(carpeta, archivo)
    if os.path.isfile(ruta):
        for linea in open(ruta, encoding='utf-8', errors='ignore'):
            if linea.startswith(clave):
                return linea.split(':', 1)[1].strip()
    return ''


def main() -> int:
    """Punto de entrada."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('carpetas', nargs='+')
    parser.add_argument('--csv', default='')
    args = parser.parse_args()

    filas = []
    for carpeta in args.carpetas:
        datos = leer_corrida(carpeta)
        fila = {
            'etiqueta': _campo(carpeta, 'entorno.txt', 'etiqueta') or carpeta,
            'perfil': _campo(carpeta, 'entorno.txt', 'perfil'),
            'codigo_secuencia': _campo(carpeta, 'secuencia.txt', 'codigo_salida'),
            **summarize_run(datos['js_t'], datos['js_q'], datos['js_names'],
                            datos['err_rows'], datos['err_names']),
            **datos['eventos'],
        }
        if not datos['err_rows']:
            print(f'  ⚠ {carpeta}: sin estado del controlador; el error de seguimiento '
                  'queda en 0 (¿se grabó con --sin-movimiento o cambió el tópico?)')
        filas.append(fila)

    columnas = list(filas[0])
    ancho = max(len(c) for c in columnas)
    print()
    for columna in columnas:
        valores = []
        for fila in filas:
            v = fila[columna]
            valores.append(f'{v:>14.4f}' if isinstance(v, float) else f'{str(v):>14}')
        print(f'{columna:<{ancho}}' + ''.join(valores))
    print('\nerror_* en rad; jerk_* en rad/s³; intervalos en ms. Un enlace que sostiene el')
    print('control deja el error y el jerk de E2/E3 cerca de E1 y sin metas abortadas.')

    if args.csv:
        with open(args.csv, 'w', newline='', encoding='utf-8') as f:
            escritor = csv.DictWriter(f, fieldnames=columnas)
            escritor.writeheader()
            escritor.writerows(filas)
        print(f'\nCSV: {args.csv}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
