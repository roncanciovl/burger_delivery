#!/usr/bin/env python3
# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""
Re-vendoriza el Kinova Gen3 de burger_description a 6 GDL (TODO.md §4.1).

Sustituye en los URDF del paquete la cadena del brazo de 7 GDL
(``gen3_half_arm_1_link``/``gen3_half_arm_2_link``, ``gen3_joint_7``, mallas de
``arms/gen3/7dof``) por la del macro oficial de Kinova para 6 GDL
(``gen3_bicep_link``, mallas de ``arms/gen3/6dof``), y copia esas mallas al paquete.

Lo que NO toca: la pinza Robotiq 2F-85 (cuelga de ``gen3_end_effector_link``, que sigue
existiendo), la cámara de muñeca (cuelga de ``gen3_bracelet_link``, que también sigue),
la mesa, los tags y el resto de la escena.

El macro de Kinova es xacro; aquí se evalúa lo mínimo, sin ROS: ``${prefix}``,
``${dof}``, los ``xacro:if``/``xacro:unless`` de ``use_external_cable`` y ``vision``,
y ``${-2*pi}``/``${2*pi}``. Si el macro trae una construcción que este script no
entiende, falla en vez de escribir un URDF a medias.

Uso::

    git clone --depth 1 https://github.com/Kinovarobotics/ros2_kortex.git /tmp/ros2_kortex
    python3 scripts/revendorizar_gen3_6dof.py /tmp/ros2_kortex/kortex_description

Es idempotente: sobre un URDF ya convertido vuelve a escribir la misma cadena.
"""

import argparse
import math
import os
import re
import shutil
import sys
import xml.etree.ElementTree as ET

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PAQUETE = os.path.join(REPO, 'burger_description')
URDFS = [
    os.path.join(PAQUETE, 'urdf', 'delivery_scene_fixed.urdf'),
    os.path.join(PAQUETE, 'urdf', 'burger_delivery_gen3.urdf'),
]
MALLAS_DESTINO = os.path.join(PAQUETE, 'vendor', 'kortex_description', 'arms', 'gen3', '6dof',
                              'meshes')
URI_MALLAS = 'package://burger_description/vendor/kortex_description/arms/gen3/6dof/meshes/'
XACRO = '{http://ros.org/wiki/xacro}'
PREFIJO = 'gen3_'


def _evaluar(texto, variables):
    """Sustituir las expresiones ${...} que usa el macro del brazo."""
    def reemplazo(m):
        expr = m.group(1).strip()
        if expr in variables:
            return str(variables[expr])
        simple = {'-2*pi': -2 * math.pi, '2*pi': 2 * math.pi}
        if expr in simple:
            return repr(simple[expr])
        raise ValueError(f'expresión xacro no soportada: ${{{expr}}}')
    return re.sub(r'\$\{([^}]*)\}', reemplazo, texto)


def _expandir(elemento, variables, salida):
    """Aplanar el cuerpo del macro en ``salida`` (lista de elementos URDF)."""
    for hijo in list(elemento):
        tag = hijo.tag
        if tag in (XACRO + 'if', XACRO + 'unless'):
            valor = _evaluar(hijo.get('value'), variables).strip().lower() == 'true'
            if valor == (tag == XACRO + 'if'):
                _expandir(hijo, variables, salida)
            continue
        if tag == XACRO + 'insert_block':
            # El bloque *origin del macro: el brazo se monta en el origen de 'world'.
            salida.append(ET.Element('origin', {'xyz': '0 0 0', 'rpy': '0 0 0'}))
            continue
        if tag in (XACRO + 'include', XACRO + 'kortex_ros2_control'):
            continue      # el bloque ros2_control del URDF de la escena se conserva
        if tag.startswith(XACRO):
            raise ValueError(f'construcción xacro no soportada: {tag}')
        nuevo = ET.Element(tag, {k: _evaluar(v, variables) for k, v in hijo.attrib.items()})
        salida.append(nuevo)
        _expandir(hijo, variables, _Collector(nuevo))


class _Collector(list):
    """Lista que además cuelga cada elemento de un padre ElementTree."""

    def __init__(self, padre):
        super().__init__()
        self.padre = padre

    def append(self, elemento):
        super().append(elemento)
        self.padre.append(elemento)


def cadena_6dof(macro_path, vision):
    """Devolver los elementos <link>/<joint> del brazo de 6 GDL ya evaluados."""
    raiz = ET.parse(macro_path).getroot()
    macro = raiz.find(XACRO + 'macro')
    if macro is None or macro.get('name') != 'load_arm':
        raise ValueError(f'{macro_path}: no se encontró el macro load_arm')
    variables = {'prefix': PREFIJO, 'dof': 6, 'parent': 'world',
                 'use_external_cable': 'false', 'vision': str(vision).lower()}
    salida = []
    _expandir(macro, variables, salida)
    elementos = [e for e in salida if e.tag in ('link', 'joint')]
    for malla in (m for e in elementos for m in e.iter('mesh')):
        nombre = os.path.basename(malla.get('filename'))
        malla.set('filename', URI_MALLAS + nombre)
    nombres = [e.get('name') for e in elementos if e.tag == 'joint' and e.get('type') != 'fixed']
    if nombres != [f'{PREFIJO}joint_{i}' for i in range(1, 7)]:
        raise ValueError(f'el macro no produjo joint_1..joint_6: {nombres}')
    return elementos


def _serializar(elementos, sangria='  '):
    lineas = []
    for e in elementos:
        ET.indent(e, space='  ', level=1)
        lineas.append(sangria + ET.tostring(e, encoding='unicode').rstrip())
    return '\n'.join(lineas) + '\n'


def convertir(urdf_path, elementos):
    """Reemplazar en un URDF la cadena del brazo y el bloque ros2_control de joint_7."""
    texto = open(urdf_path, encoding='utf-8').read()
    inicio = re.search(r'^[ \t]*<joint name="gen3_base_joint"', texto, re.M)
    fin = re.search(r'^[ \t]*<link name="gen3_robotiq_85_base_link"', texto, re.M)
    if not inicio or not fin or fin.start() < inicio.start():
        raise ValueError(f'{urdf_path}: no se encontraron los límites de la cadena del brazo')
    texto = texto[:inicio.start()] + _serializar(elementos) + texto[fin.start():]
    # El bloque ros2_control describe las interfaces del hardware: joint_7 sobra.
    texto, n = re.subn(
        r'^[ \t]*<joint name="gen3_joint_7">.*?</joint>\n', '', texto, flags=re.S | re.M)
    ET.fromstring(texto)  # debe seguir siendo XML válido
    with open(urdf_path, 'w', encoding='utf-8') as salida:
        salida.write(texto)
    return n


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('kortex_description', help='carpeta kortex_description de ros2_kortex')
    parser.add_argument('--vision', action='store_true',
                        help='usar bracelet_with_vision (por defecto, igual que el modelo '
                             'anterior: bracelet_no_vision + cámara de muñeca propia)')
    args = parser.parse_args()

    brazo = os.path.join(args.kortex_description, 'arms', 'gen3', '6dof')
    elementos = cadena_6dof(os.path.join(brazo, 'urdf', 'gen3_macro.xacro'), args.vision)

    os.makedirs(MALLAS_DESTINO, exist_ok=True)
    usadas = sorted({os.path.basename(m.get('filename'))
                     for e in elementos for m in e.iter('mesh')})
    for nombre in usadas:
        shutil.copy2(os.path.join(brazo, 'meshes', nombre), os.path.join(MALLAS_DESTINO, nombre))
    print(f'mallas copiadas a {os.path.relpath(MALLAS_DESTINO, REPO)}: {", ".join(usadas)}')

    for urdf in URDFS:
        n = convertir(urdf, elementos)
        print(f'{os.path.relpath(urdf, REPO)}: cadena de 6 GDL escrita '
              f'({n} bloque ros2_control de joint_7 eliminado)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
