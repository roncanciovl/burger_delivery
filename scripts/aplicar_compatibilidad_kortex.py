#!/usr/bin/env python3
# Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
# SPDX-License-Identifier: Apache-2.0
"""
Ajustes de compatibilidad obligatorios sobre `ros2_kortex` recién clonado.

Sustituye a `apply_kinova_smooth_movement.py`. Aquel script nació de un experimento no
documentado: el brazo temblaba al moverse y se atribuyó a la latencia del router UDP de la
API Kortex, reduciendo su timeout. **Esa premisa quedó descartada**: el temblor venía del
enlace de red de la estación. Medido sobre el robot real, con la misma máquina y cambiando
sólo WiFi por cable, el intervalo p99 de `/joint_states` cayó de 60.12 ms a 10.61 ms y las
pérdidas de telemetría de 2 a 0. Ver
`burger_kinova_reference/docs/EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md`.

Aquel parche además **no aplicaba nada**: buscaba `SetMessageTimeout(500)`, patrón que ya
no existe en la versión actual de `ros2_kortex`, y aun así imprimía `[x] Parcheado`.

Lo que este script SÍ hace, y por qué:

1. **Retira los argumentos de simulación de los xacros.** El `robotiq_description` que
   instala apt en Jazzy no acepta `mock_sensor_commands`, `sim_gazebo`,
   `isaac_joint_commands` ni `isaac_joint_states`, que es lo que le envía el macro de
   Kinova. Sin esto el xacro no se genera y **ningún nodo arranca**.

2. **Elimina los bloques `<xacro:if>` que quedan huérfanos.** Éste es el arreglo
   importante frente al script anterior: al borrar los parámetros pero dejar los bloques,
   quedaban expresiones `${}` vacías y el xacro fallaba con
   `error: invalid syntax (<expression>, line 0)`, un mensaje que no señala la causa. Así
   se inutilizó la descripción de 6 GDL —la de este robot— y se acabó usando `dof:=7`
   sobre un brazo de seis actuadores.

3. **Baja `update_rate` a 100 Hz** en la configuración de controladores. A 1000 Hz el
   `controller_manager` no sostiene el ciclo bajo WSL2 y desborda continuamente.

4. **Activa el bus interno de la pinza.** No afecta al árbol TF —links y joints quedan
   idénticos, comprobado—; sólo añade las interfaces `ros2_control` del gripper.

A diferencia del anterior, **verifica cada cambio y falla ruidosamente** si algo no se
pudo aplicar, en lugar de reportar éxito. Es idempotente: volver a ejecutarlo no rompe
nada.

Uso::

    python3 scripts/aplicar_compatibilidad_kortex.py            # aplica
    python3 scripts/aplicar_compatibilidad_kortex.py --check    # sólo comprueba
"""

import argparse
import os
import re
import sys
from typing import List, Optional, Tuple

#: Argumentos de simulación que el robotiq_description de la distribución no acepta.
PARAMS_SIMULACION = (
    'sim_gazebo', 'sim_isaac', 'mock_sensor_commands',
    'isaac_joint_commands', 'isaac_joint_states',
)

def listar_xacros(raiz: str) -> List[str]:
    """
    Enumerar todos los xacro de la descripción.

    Se recorre el árbol en lugar de mantener una lista fija: la primera versión de este
    script llevaba ocho rutas escritas a mano y dejaba fuera cinco archivos que también
    envían argumentos de simulación —entre ellos el macro del brazo de 7 GDL—, de modo
    que el árbol quedaba a medio ajustar. Recorrerlo evita que la lista envejezca cuando
    Kinova reorganice el paquete.

    :param raiz: directorio de ``kortex_description``.
    :returns: rutas absolutas de los xacro, en orden estable.
    """
    encontrados = []
    for base, _, archivos in os.walk(raiz):
        for archivo in sorted(archivos):
            if archivo.endswith('.xacro'):
                encontrados.append(os.path.join(base, archivo))
    return sorted(encontrados)

CONTROLADORES = (
    'arms/gen3/6dof/config/ros2_controllers.yaml',
    'arms/gen3/7dof/config/ros2_controllers.yaml',
)


def quitar_params_simulacion(texto: str) -> str:
    """
    Retirar de un xacro los argumentos de simulación y sus usos.

    :param texto: contenido del archivo.
    :returns: el contenido sin esos argumentos.
    """
    for nombre in PARAMS_SIMULACION:
        # Declaración en la firma del macro, con o sin valor por defecto.
        texto = re.sub(rf'^\s*{nombre}(:=[^\s]*)?\s*$\n?', '', texto, flags=re.M)
        # Paso del argumento a otro macro.
        texto = re.sub(rf'\s*{nombre}="[^"]*"', '', texto)
        texto = re.sub(rf'\s*{nombre}:="[^"]*"', '', texto)
        # <param name="..."> del bloque ros2_control.
        texto = re.sub(rf'^\s*<param\s+name="{nombre}">[^<]*</param>\s*$\n?', '',
                       texto, flags=re.M)
        # <xacro:arg name="..." default="..." /> de nivel superior. Se retira sólo si
        # nadie la consume: si quedara un $(arg ...) vivo, xacro fallaría al resolverlo.
        if not re.search(rf'\$\(arg\s+{nombre}\s*\)', texto):
            texto = re.sub(
                rf'^\s*<xacro:arg\s+name="{nombre}"[^/]*/>\s*$\n?', '',
                texto, flags=re.M)
    # Términos sueltos dentro de expresiones booleanas.
    for nombre in ('sim_gazebo', 'sim_isaac'):
        texto = re.sub(rf'\s+or\s+{nombre}\b', '', texto)
        texto = re.sub(rf'\b{nombre}\s+or\s+', '', texto)
    return texto


def _condicion_es_de_simulacion(expresion: str) -> bool:
    """
    Decidir si una condición xacro depende **sólo** de argumentos de simulación.

    Cubre las formas que aparecen en la práctica: ``${sim_gazebo}``,
    ``${sim_gazebo or sim_isaac}``, ``$(arg sim_gazebo)`` y ``${}`` —este último, el
    residuo que dejaba el script anterior—. Una condición que mezcle una variable de
    simulación con otra cosa NO se considera de simulación y se deja intacta, porque
    eliminarla cambiaría el comportamiento en hardware real.

    :param expresion: contenido de la condición, sin la envoltura ``${}`` o ``$(arg )``.
    :returns: ``True`` si sólo referencia argumentos de simulación (o está vacía).
    """
    resto = expresion
    for nombre in PARAMS_SIMULACION:
        resto = re.sub(rf'\b{nombre}\b', '', resto)
    resto = re.sub(r'\b(or|and|not)\b', '', resto)
    return resto.strip(' ()') == ''


def _extraer_condicion(valor: str) -> Optional[str]:
    """
    Obtener la expresión de un atributo ``value`` de xacro.

    :param valor: contenido del atributo, p. ej. ``${sim_gazebo}`` o ``$(arg sim_isaac)``.
    :returns: la expresión interna, o ``None`` si no es una de las dos formas.
    """
    llaves = re.fullmatch(r'\$\{([^}]*)\}', valor.strip())
    if llaves:
        return llaves.group(1)
    argumento = re.fullmatch(r'\$\(arg\s+([^)]*)\)', valor.strip())
    if argumento:
        return argumento.group(1)
    return None


_APERTURA = re.compile(r'<xacro:(if|unless)\s+value="([^"]*)"\s*>')


def _fin_del_bloque(texto: str, desde: int) -> int:
    """
    Localizar el cierre que corresponde a un bloque, contando anidamiento.

    :param texto: contenido completo.
    :param desde: índice justo después de la etiqueta de apertura.
    :returns: índice donde empieza la etiqueta de cierre correspondiente, o ``-1``.
    """
    patron = re.compile(r'<xacro:(if|unless)\b[^>]*>|</xacro:(if|unless)>')
    profundidad = 1
    posicion = desde
    while profundidad > 0:
        match = patron.search(texto, posicion)
        if match is None:
            return -1
        if match.group(0).startswith('</'):
            profundidad -= 1
            if profundidad == 0:
                return match.start()
        elif not match.group(0).endswith('/>'):
            profundidad += 1
        posicion = match.end()
    return -1


def limpiar_bloques_huerfanos(texto: str) -> Tuple[str, int, int]:
    """
    Resolver los bloques condicionados a argumentos de simulación.

    Un ``<xacro:if>`` cuya condición sólo se cumplía en simulación nunca se cumple sin
    ella: el bloque **se elimina entero**, con lo que tenga anidado dentro. Un
    ``<xacro:unless>`` es el complemento —se cumplía siempre fuera de simulación— así que
    **se conserva su contenido** y se retira sólo la envoltura.

    Se recorre el texto contando anidamiento en lugar de usar una expresión regular. Las
    dos formas de anidamiento que aparecen en `ros2_kortex` derrotan a cualquier regex
    razonable: en `gen3_macro.xacro` el bloque de simulación va **dentro** de un
    ``<xacro:if value="${vision}">``, y en `gen3_lite` el bloque de simulación
    **contiene** un ``<xacro:unless value="${moveit_active}">``. Un escáner con
    profundidad resuelve ambas sin depender del orden.

    Es la diferencia decisiva frente al script anterior, que borraba los argumentos pero
    dejaba los bloques. Quedaban expresiones ``${}`` vacías y xacro fallaba con
    ``invalid syntax (<expression>, line 0)``, un mensaje que no señala la causa; así se
    inutilizó la descripción de 6 GDL, la de este robot. Borrar sólo el nombre de la
    variable tampoco sirve: dejaría ``${sim_gazebo}`` sin declarar y xacro fallaría
    igualmente, con otro mensaje.

    :param texto: contenido del archivo.
    :returns: tupla ``(contenido, bloques_if_eliminados, envolturas_unless_abiertas)``.
    """
    n_if = 0
    n_unless = 0
    posicion = 0
    while True:
        apertura = _APERTURA.search(texto, posicion)
        if apertura is None:
            break
        condicion = _extraer_condicion(apertura.group(2))
        if condicion is None or not _condicion_es_de_simulacion(condicion):
            # No es de simulación: se entra en él, para poder tratar lo que anide dentro.
            posicion = apertura.end()
            continue

        cierre = _fin_del_bloque(texto, apertura.end())
        if cierre < 0:
            posicion = apertura.end()
            continue
        etiqueta = apertura.group(1)
        fin_cierre = texto.index('>', cierre) + 1
        cuerpo = texto[apertura.end():cierre]

        # Se recorta la línea completa de apertura y de cierre para no dejar indentación
        # huérfana ni líneas en blanco.
        arranque = texto.rfind('\n', 0, apertura.start()) + 1
        remate = fin_cierre
        if texto[remate:remate + 1] == '\n':
            remate += 1

        if etiqueta == 'if':
            reemplazo = ''
            n_if += 1
        else:
            reemplazo = cuerpo.strip('\n') + '\n' if cuerpo.strip() else ''
            n_unless += 1
        texto = texto[:arranque] + reemplazo + texto[remate:]
        posicion = arranque
    return texto, n_if, n_unless


def activar_bus_pinza(texto: str) -> str:
    """Poner a ``true`` el valor por defecto de ``use_internal_bus_gripper_comm``."""
    return re.sub(
        r'<xacro:arg\s+name="use_internal_bus_gripper_comm"\s+default="false"\s*/>',
        '<xacro:arg name="use_internal_bus_gripper_comm" default="true" />', texto)


def procesar_xacro(ruta: str, aplicar: bool, raiz: str = '') -> List[str]:
    """
    Aplicar (o comprobar) los ajustes sobre un xacro.

    :param ruta: ruta del archivo.
    :param aplicar: si es ``False``, sólo informa de lo que haría.
    :param raiz: directorio base, para informar con rutas legibles.
    :returns: lista de mensajes de lo hecho o pendiente.
    """
    with open(ruta, encoding='utf-8') as fh:
        original = fh.read()

    # Primero los bloques, que todavía referencian las variables por su nombre; después
    # las declaraciones. Al revés quedarían referencias a argumentos ya inexistentes.
    texto, n_if, n_unless = limpiar_bloques_huerfanos(original)
    texto = quitar_params_simulacion(texto)
    if ruta.endswith(('gen3.xacro', 'kortex_robot.xacro')):
        texto = activar_bus_pinza(texto)

    if texto == original:
        return []
    detalle = []
    if n_if:
        detalle.append(f'{n_if} bloque(s) <xacro:if> huérfano(s) eliminado(s)')
    if n_unless:
        detalle.append(f'{n_unless} envoltura(s) <xacro:unless> desenvuelta(s)')
    detalle.append('argumentos de simulación retirados')
    if aplicar:
        with open(ruta, 'w', encoding='utf-8') as fh:
            fh.write(texto)
    nombre = os.path.relpath(ruta, raiz) if raiz else os.path.basename(ruta)
    return [f"{'✔' if aplicar else '·'} {nombre}: {', '.join(detalle)}"]


def procesar_controladores(ruta: str, aplicar: bool, raiz: str = '') -> List[str]:
    """Bajar ``update_rate`` a 100 Hz."""
    with open(ruta, encoding='utf-8') as fh:
        original = fh.read()
    texto = re.sub(r'update_rate:\s*1000\b', 'update_rate: 100', original)
    if texto == original:
        return []
    if aplicar:
        with open(ruta, 'w', encoding='utf-8') as fh:
            fh.write(texto)
    nombre = os.path.relpath(ruta, raiz) if raiz else os.path.basename(ruta)
    return [f"{'✔' if aplicar else '·'} {nombre}: update_rate 1000 -> 100 Hz"]


def verificar(raiz: str) -> List[str]:
    """
    Comprobar que no queda nada que impida generar los URDF.

    :param raiz: directorio de ``kortex_description``.
    :returns: lista de problemas encontrados; vacía si todo está correcto.
    """
    problemas: List[str] = []
    for base, _, archivos in os.walk(raiz):
        for archivo in archivos:
            if not archivo.endswith('.xacro'):
                continue
            ruta = os.path.join(base, archivo)
            with open(ruta, encoding='utf-8') as fh:
                contenido = fh.read()
            if '${}' in contenido:
                n = contenido.count('${}')
                problemas.append(
                    f'{os.path.relpath(ruta, raiz)}: quedan {n} expresión(es) "${{}}" '
                    f'vacía(s); xacro fallará con "invalid syntax (<expression>, line 0)"')
            for nombre in PARAMS_SIMULACION:
                if re.search(rf'\b{nombre}\b', contenido):
                    problemas.append(
                        f'{os.path.relpath(ruta, raiz)}: todavía referencia "{nombre}". '
                        f'Si es un argumento que se envía, el robotiq_description de la '
                        f'distribución lo rechaza; si es una condición, xacro fallará '
                        f'porque ya no está declarado')
    for relativo in CONTROLADORES:
        ruta = os.path.join(raiz, relativo)
        if os.path.isfile(ruta):
            with open(ruta, encoding='utf-8') as fh:
                if re.search(r'update_rate:\s*1000\b', fh.read()):
                    problemas.append(
                        f'{relativo}: update_rate sigue en 1000 Hz; el controller_manager '
                        f'desbordará el ciclo bajo WSL2')
    return problemas


def main() -> int:
    """Punto de entrada."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--check', action='store_true',
                        help='sólo comprobar, sin modificar nada')
    parser.add_argument('--workspace-src', default=None,
                        help='ruta a ~/ros2_ws/src (por defecto se deduce del script)')
    args = parser.parse_args()

    if args.workspace_src:
        src = os.path.abspath(args.workspace_src)
    else:
        aqui = os.path.dirname(os.path.abspath(__file__))
        src = os.path.abspath(os.path.join(aqui, '..', '..'))

    descripcion = os.path.join(src, 'ros2_kortex', 'kortex_description')
    if not os.path.isdir(descripcion):
        print(f'[ERROR] No se encontró kortex_description en {descripcion}.')
        print('        Clona ros2_kortex en ~/ros2_ws/src antes de ejecutar este script.')
        return 2

    aplicar = not args.check
    print('--- Ajustes de compatibilidad sobre ros2_kortex '
          f"({'aplicando' if aplicar else 'sólo comprobando'}) ---")

    hechos: List[str] = []
    faltantes: List[str] = []
    for ruta in listar_xacros(descripcion):
        hechos.extend(procesar_xacro(ruta, aplicar, descripcion))
    for relativo in CONTROLADORES:
        ruta = os.path.join(descripcion, relativo)
        if not os.path.isfile(ruta):
            faltantes.append(relativo)
            continue
        hechos.extend(procesar_controladores(ruta, aplicar, descripcion))

    for linea in hechos:
        print(f'  {linea}')
    if not hechos:
        print('  (nada que cambiar: los ajustes ya estaban aplicados)')
    for relativo in faltantes:
        print(f'  ⚠ no encontrado, se omite: {relativo}')

    problemas = verificar(descripcion)
    if problemas:
        print('\n[FALLO] La comprobación posterior encontró problemas:')
        for problema in problemas:
            print(f'  ✗ {problema}')
        print('\nNo se declara éxito. Revisa los archivos señalados: si la versión de '
              'ros2_kortex cambió, los patrones de este script pueden haber quedado '
              'obsoletos.')
        return 1

    print('\n[OK] Comprobación superada: no quedan expresiones vacías, ni argumentos '
          'que el robotiq_description rechace, ni update_rate a 1000 Hz.')
    if aplicar and hechos:
        print('Recompila:  colcon build --symlink-install '
              '--packages-select kortex_description kortex_driver')
    return 0


if __name__ == '__main__':
    sys.exit(main())
