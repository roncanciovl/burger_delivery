#!/usr/bin/env python3
# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""
Validación estructural de los URDF del repositorio, sin ROS instalado.

Complementa a ``check_urdf`` (liburdfdom-tools) con lo que ese comando NO revisa y que
en este proyecto ya produjo fallos silenciosos (TROUBLESHOOTING.md §4.5):

1. XML bien formado y raíz ``<robot>``.
2. Nombres de ``link`` y ``joint`` únicos.
3. Cada ``joint`` apunta a un ``parent`` y un ``child`` que existen.
4. Árbol válido: una sola raíz, ningún link con dos padres, sin ciclos.
5. Toda malla referenciada existe en el árbol de fuentes:
   * ``package://<paquete>/ruta`` se resuelve contra la carpeta del paquete en el repo
     (se siguen los enlaces simbólicos versionados, p. ej. ``meshes -> visual/meshes``);
   * una ruta relativa se resuelve contra la carpeta del propio URDF (visor web).
6. Expresiones xacro sin evaluar (``${...}``) fuera de ``<ros2_control>``: un ``.urdf``
   con ``${pi/2}`` en un ``origin`` lo rechaza ``robot_state_publisher``.

Uso::

    python3 scripts/ci/validar_urdf.py                   # todos los *.urdf versionados
    python3 scripts/ci/validar_urdf.py archivo.urdf ...  # sólo los indicados

Sale con código 1 si algún archivo tiene errores; imprime una línea por problema.
"""

import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def _paquetes(repo: str) -> Dict[str, str]:
    """Mapear nombre de paquete ROS -> carpeta, leyendo cada package.xml del repo."""
    paquetes = {}
    for raiz, dirs, archivos in os.walk(repo):
        dirs[:] = [d for d in dirs if d not in ('.git', 'build', 'install', 'log')]
        if 'package.xml' in archivos:
            try:
                nombre = ET.parse(os.path.join(raiz, 'package.xml')).findtext('name')
            except ET.ParseError:
                continue
            if nombre:
                paquetes[nombre.strip()] = raiz
    return paquetes


def resolver_malla(uri: str, urdf_dir: str, paquetes: Dict[str, str]) -> Optional[str]:
    """
    Convertir la URI de una malla en una ruta del árbol de fuentes.

    :returns: la ruta absoluta, o ``None`` si la URI apunta a un paquete ajeno al repo
        (p. ej. ``package://kortex_description``), que no se puede comprobar aquí.
    """
    if uri.startswith('package://'):
        paquete, _, resto = uri[len('package://'):].partition('/')
        if paquete not in paquetes:
            return None
        return os.path.join(paquetes[paquete], resto)
    if uri.startswith('file://'):
        return uri[len('file://'):]
    return os.path.normpath(os.path.join(urdf_dir, uri))


def validar(ruta: str, paquetes: Dict[str, str]) -> List[str]:
    """Validar un URDF y devolver la lista de problemas encontrados."""
    try:
        arbol = ET.parse(ruta)
    except ET.ParseError as error:
        return [f'XML mal formado: {error}']
    robot = arbol.getroot()
    if robot.tag != 'robot':
        return [f'la raíz es <{robot.tag}>, se esperaba <robot>']

    problemas = []
    links = [link.get('name') for link in robot.findall('link')]
    # Sólo las articulaciones cinemáticas (hijas directas de <robot>); las de
    # <ros2_control> y <transmission> repiten nombres a propósito.
    joints = robot.findall('joint')

    for tipo, nombres in (('link', links), ('joint', [j.get('name') for j in joints])):
        vistos = set()
        for nombre in nombres:
            if not nombre:
                problemas.append(f'{tipo} sin atributo name')
            elif nombre in vistos:
                problemas.append(f'{tipo} duplicado: {nombre}')
            vistos.add(nombre)

    conjunto = set(links)
    padre_de: Dict[str, str] = {}
    for joint in joints:
        nombre = joint.get('name')
        padre = joint.find('parent')
        hijo = joint.find('child')
        if padre is None or hijo is None:
            problemas.append(f'joint {nombre}: falta <parent> o <child>')
            continue
        p, h = padre.get('link'), hijo.get('link')
        for rol, link in (('parent', p), ('child', h)):
            if link not in conjunto:
                problemas.append(f'joint {nombre}: {rol} "{link}" no está definido')
        if h in padre_de:
            problemas.append(f'link {h} tiene dos padres ({padre_de[h]} y {p})')
        padre_de[h] = p

    raices = [link for link in links if link not in padre_de]
    if len(raices) != 1:
        problemas.append(f'se esperaba una sola raíz y hay {len(raices)}: {raices[:6]}')

    for link in links:
        visto, actual = set(), link
        while actual in padre_de:
            if actual in visto:
                problemas.append(f'ciclo en la cadena que pasa por {link}')
                break
            visto.add(actual)
            actual = padre_de[actual]

    urdf_dir = os.path.dirname(os.path.abspath(ruta))
    faltantes = set()
    for malla in robot.iter('mesh'):
        uri = malla.get('filename', '')
        destino = resolver_malla(uri, urdf_dir, paquetes)
        if destino is not None and not os.path.isfile(destino):
            faltantes.add(uri)
    problemas.extend(f'malla inexistente: {uri}' for uri in sorted(faltantes))

    control = {id(e) for c in robot.findall('ros2_control') for e in c.iter()}
    for elemento in robot.iter():
        if id(elemento) in control:
            continue
        for atributo, valor in elemento.attrib.items():
            if '${' in valor or '$(' in valor:
                problemas.append(
                    f'expresión xacro sin evaluar en <{elemento.tag} {atributo}="{valor}">')
    return problemas


def _urdf_versionados() -> List[str]:
    salida = subprocess.run(
        ['git', 'ls-files', '*.urdf'], cwd=REPO, capture_output=True, text=True, check=True)
    return [os.path.join(REPO, linea) for linea in salida.stdout.split()]


def main(argv: List[str]) -> int:
    archivos = argv or _urdf_versionados()
    paquetes = _paquetes(REPO)
    total = 0
    for ruta in archivos:
        problemas = validar(ruta, paquetes)
        relativa = os.path.relpath(ruta, REPO)
        if problemas:
            for problema in problemas:
                print(f'✗ {relativa}: {problema}')
        else:
            print(f'✓ {relativa}')
        total += len(problemas)
    print(f'\n{len(archivos)} archivos, {total} problemas')
    return 1 if total else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
