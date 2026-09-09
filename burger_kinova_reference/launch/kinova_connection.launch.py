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
Launch unificado del enlace con el Kinova Gen3 (RF-02).

Un único archivo cubre los tres modos de operación exigidos por la especificación:

* **Validación sin robot** (modo fake, driver local)::

      ros2 launch burger_kinova_reference kinova_connection.launch.py \\
          start_driver:=true robot_ip:=0.0.0.0 use_fake_hardware:=true \\
          enable_motion:=false

* **Driver y monitor en la misma estación** (estación A, conectada al robot)::

      ros2 launch burger_kinova_reference kinova_connection.launch.py \\
          start_driver:=true robot_ip:=192.168.1.10 use_fake_hardware:=false \\
          enable_motion:=false

* **Monitor en una segunda estación** (estación B, cliente por DDS)::

      export ROS_DOMAIN_ID=<dominio_del_equipo>
      ros2 launch burger_kinova_reference kinova_connection.launch.py \\
          start_driver:=false enable_motion:=false

⚠ Sólo la estación conectada físicamente al robot puede usar ``start_driver:=true``.
Dos drivers apuntando al mismo Kinova compiten por la única sesión de control en tiempo
real de la API Kortex y duplican ``/joint_states``, ``/controller_manager`` y el servidor
de acción dentro del mismo ``ROS_DOMAIN_ID``.

Los valores por defecto de cada argumento se leen de ``config/kinova_connection.yaml``,
de modo que no hay constantes duplicadas entre la configuración y el launch.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    Shutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import yaml

PACKAGE_NAME = 'burger_kinova_reference'


def _load_launch_defaults(config_path):
    """
    Leer la sección ``launch`` del YAML de configuración.

    :param config_path: ruta al archivo de configuración.
    :returns: diccionario de valores por defecto (vacío si el archivo no es legible).
    """
    try:
        with open(config_path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
    except (OSError, yaml.YAMLError):
        return {}
    return (data.get('launch') or {}).get('ros__parameters', {}) or {}


def _as_bool(value) -> bool:
    """
    Convertir a booleano un valor proveniente de la línea de comandos o del YAML.

    :param value: valor a interpretar.
    :returns: su equivalente booleano.
    """
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('true', '1', 'yes', 'si', 'sí', 'on')


#: Pinzas cuyo bloque ``ros2_control`` de ros2_kortex declara articulaciones ``mimic``
#: CON ``command_interface`` en la rama de hardware simulado. ROS 2 Jazzy aborta el
#: ``ros2_control_node`` ante esa combinación (ver :func:`_resolve_gripper`).
_GRIPPERS_ROTOS_EN_FAKE = ('robotiq_2f_85', 'robotiq_2f_140')

#: Valores que significan "sin pinza" en la línea de comandos.
_SIN_PINZA = ('', 'none', 'ninguna', 'sin_pinza', 'false')


def _resolve_gripper(gripper: str, use_fake_hardware: bool, force: bool):
    """
    Decidir qué pinza se transfiere al bringup y por qué.

    En **hardware real** el bloque ``ros2_control`` del Robotiq 2F-85 no expone
    ``command_interface`` sobre sus articulaciones ``mimic``, y el driver arranca sin
    problemas. En **hardware simulado** el mismo xacro conmuta a
    ``mock_components/GenericSystem`` y sí declara ``command_interface`` en las cinco
    articulaciones ``mimic`` del gripper. ROS 2 Jazzy lo rechaza y aborta el proceso::

        terminate called after throwing an instance of 'std::runtime_error'
          what():  Joint 'robotiq_85_right_knuckle_joint' has mimic attribute not set
                   to false: Activated mimic joints cannot have command interfaces.

    Es una incompatibilidad de ``ros2_kortex`` con Jazzy, no de este package, y no puede
    corregirse aquí: el requisito de calidad prohíbe modificar archivos dentro de
    ``ros2_kortex``. Como la pinza está además fuera del alcance del corte 1, en modo
    fake se omite y el brazo de 6 GDL se valida completo.

    :param gripper: valor solicitado del argumento ``gripper``.
    :param use_fake_hardware: ``True`` si se opera con hardware simulado.
    :param force: forzar la pinza en modo fake aunque se sepa que aborta el driver.
    :returns: tupla ``(pinza_efectiva, aviso)``; la pinza vacía significa "sin pinza" y
        hace que no se transfiera el argumento al bringup.
    """
    solicitada = (gripper or '').strip()
    if solicitada.lower() in _SIN_PINZA:
        return '', ''
    if not use_fake_hardware or force:
        return solicitada, ''
    if solicitada.lower() not in _GRIPPERS_ROTOS_EN_FAKE:
        return solicitada, ''
    aviso = (
        f'⚠ gripper:={solicitada} se omite en modo fake. El bloque ros2_control de esa '
        f'pinza en ros2_kortex declara articulaciones mimic con command_interface, y '
        f'ROS 2 Jazzy aborta el ros2_control_node con "Activated mimic joints cannot '
        f'have command interfaces". Se valida el brazo de 6 GDL sin pinza (la pinza '
        f'está fuera del alcance del corte 1). Con hardware real la pinza SÍ se '
        f'transfiere. Para forzarla de todos modos: force_gripper_in_fake:=true'
    )
    return '', aviso


def _setup(context, *args, **kwargs):
    """
    Construir las acciones del launch una vez resueltas las sustituciones.

    :param context: contexto de launch con los argumentos ya resueltos.
    :returns: lista de acciones a ejecutar.
    """
    del args, kwargs

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    config_file = arg('config_file')
    start_driver = _as_bool(arg('start_driver'))
    robot_ip = arg('robot_ip')
    use_fake_hardware = _as_bool(arg('use_fake_hardware'))
    launch_rviz = _as_bool(arg('launch_rviz'))
    enable_motion = _as_bool(arg('enable_motion'))
    start_trajectory_client = _as_bool(arg('start_trajectory_client'))
    dry_run = _as_bool(arg('dry_run'))
    log_level = arg('log_level')
    gripper, gripper_aviso = _resolve_gripper(
        arg('gripper'), use_fake_hardware, _as_bool(arg('force_gripper_in_fake')))

    actions = []

    # ---------------------------------------------------------------- validaciones --
    if not os.path.isfile(config_file):
        return [
            LogInfo(msg=f'[ERROR] No se encontró el archivo de configuración: {config_file}'),
            Shutdown(reason='configuración ausente'),
        ]

    # La validación de robot_ip vive en el package (safety.validate_robot_ip) y se importa
    # aquí para que el launch falle temprano, antes de arrancar el driver.
    from burger_kinova_reference.safety import validate_robot_ip
    ip_errors = validate_robot_ip(robot_ip, start_driver, use_fake_hardware)
    if ip_errors:
        return [LogInfo(msg=f'[ERROR] {error}') for error in ip_errors] + [
            Shutdown(reason='robot_ip inválido para el modo solicitado')
        ]

    if enable_motion:
        actions.append(LogInfo(msg=(
            '⚠ enable_motion:=true — habilitación explícita de comandos físicos. '
            'Verifica espacio despejado, parada de emergencia accesible y autorización '
            'del responsable del laboratorio.'
        )))

    # -------------------------------------------------------- driver oficial Kinova --
    if start_driver:
        actions.append(LogInfo(msg=(
            f'Iniciando kortex_bringup en ESTA estación (robot_ip={robot_ip}, '
            f'use_fake_hardware={use_fake_hardware}). Ninguna otra estación del mismo '
            f'ROS_DOMAIN_ID debe lanzar el driver para el mismo robot.'
        )))
        try:
            kortex_launch = os.path.join(
                get_package_share_directory('kortex_bringup'), 'launch', 'gen3.launch.py')
        except Exception as exc:  # noqa: BLE001 - se informa la causa y se aborta el launch
            return [
                LogInfo(msg=(
                    f'[ERROR] start_driver:=true pero kortex_bringup no está disponible en '
                    f'el workspace ({exc}). Clona y compila ros2_kortex en ~/ros2_ws/src, o '
                    f'lanza con start_driver:=false para operar como cliente DDS.'
                )),
                Shutdown(reason='kortex_bringup no disponible'),
            ]
        if gripper_aviso:
            actions.append(LogInfo(msg=gripper_aviso))
        bringup_args = {
            'robot_type': arg('robot_type'),
            'robot_ip': robot_ip,
            'dof': arg('dof'),
            'use_fake_hardware': str(use_fake_hardware).lower(),
            'robot_controller': arg('robot_controller'),
            'launch_rviz': str(launch_rviz).lower(),
            # `gripper` se transfiere SIEMPRE, incluso vacío. Un launch incluido hereda
            # las configuraciones del padre, así que omitir la clave no equivale a "sin
            # pinza": gen3.launch.py vería el valor que este archivo ya declaró y su
            # propio DeclareLaunchArgument no lo sobrescribiría.
            'gripper': gripper,
        }
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kortex_launch),
            launch_arguments=bringup_args.items(),
        ))
    else:
        actions.append(LogInfo(msg=(
            'start_driver:=false — esta estación opera como CLIENTE. El driver debe estar '
            'corriendo en la estación conectada al robot, dentro del mismo ROS_DOMAIN_ID.'
        )))

    # ------------------------------------------------------------------- monitor ----
    overrides = {
        'use_fake_hardware': use_fake_hardware,
        'enable_motion': enable_motion,
        'log_level': log_level,
        # El monitor no se conecta al robot; usa estos dos para saber si ESTA estación
        # es la anfitriona del driver y anunciarlo en el diagnóstico.
        'robot_ip': robot_ip,
        'start_driver': start_driver,
    }
    actions.append(Node(
        package=PACKAGE_NAME,
        executable='kinova_monitor',
        name='kinova_monitor',
        output='screen',
        emulate_tty=True,
        parameters=[config_file, overrides],
        arguments=['--ros-args', '--log-level', f'kinova_monitor:={log_level}'],
    ))

    # -------------------------------------------------- cliente de trayectoria -----
    if start_trajectory_client:
        actions.append(Node(
            package=PACKAGE_NAME,
            executable='safe_trajectory_client',
            name='safe_trajectory_client',
            output='screen',
            emulate_tty=True,
            parameters=[config_file, dict(overrides, dry_run=dry_run)],
            arguments=['--ros-args', '--log-level', f'safe_trajectory_client:={log_level}'],
        ))

    return actions


def generate_launch_description() -> LaunchDescription:
    """
    Declarar los argumentos del launch y delegar el armado en :func:`_setup`.

    :returns: la descripción de launch completa.
    """
    default_config = os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'config', 'kinova_connection.yaml')
    defaults = _load_launch_defaults(default_config)

    declared = [
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Ruta al YAML de configuración del enlace.'),
        DeclareLaunchArgument(
            'start_driver', default_value=str(defaults.get('start_driver', False)).lower(),
            choices=['true', 'false'],
            description='Iniciar kortex_bringup en esta estación. Sólo la estación '
                        'conectada físicamente al robot puede usar true.'),
        DeclareLaunchArgument(
            'robot_ip', default_value=str(defaults.get('robot_ip', '0.0.0.0')),
            description='IP del Kinova cuando el driver se inicia localmente.'),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value=str(defaults.get('use_fake_hardware', True)).lower(),
            choices=['true', 'false'],
            description='Validar el package sin movimiento físico.'),
        DeclareLaunchArgument(
            'launch_rviz', default_value=str(defaults.get('launch_rviz', False)).lower(),
            choices=['true', 'false'],
            description='Abrir RViz como herramienta de observación.'),
        DeclareLaunchArgument(
            'enable_motion', default_value='false', choices=['true', 'false'],
            description='Habilitación explícita de comandos físicos. Siempre false por '
                        'defecto, incluso si el YAML dijera otra cosa.'),
        DeclareLaunchArgument(
            'start_trajectory_client', default_value='false', choices=['true', 'false'],
            description='Lanzar también el cliente de trayectoria segura.'),
        DeclareLaunchArgument(
            'dry_run', default_value='true', choices=['true', 'false'],
            description='El cliente valida sin enviar la meta al servidor de acción.'),
        DeclareLaunchArgument(
            'log_level', default_value=str(defaults.get('log_level', 'info')),
            choices=['debug', 'info', 'warn', 'error', 'fatal'],
            description='Nivel de log inicial. Cambiable en caliente con '
                        '"ros2 param set /kinova_monitor log_level debug".'),
        DeclareLaunchArgument(
            'robot_type', default_value=str(defaults.get('robot_type', 'gen3')),
            description='Modelo transferido a kortex_bringup.'),
        DeclareLaunchArgument(
            'dof', default_value=str(defaults.get('dof', 6)),
            description='Grados de libertad del manipulador. El brazo del laboratorio '
                        'es un Gen3 de 6 GDL: lanzarlo con 7 fabrica una articulación '
                        'que el robot no reporta.'),
        DeclareLaunchArgument(
            'gripper', default_value=str(defaults.get('gripper', 'robotiq_2f_85')),
            description='Pinza transferida a kortex_bringup. Usa "none" para el brazo '
                        'solo. En modo fake se omite automáticamente (ver la nota sobre '
                        'mimic joints en el README).'),
        DeclareLaunchArgument(
            'force_gripper_in_fake', default_value='false', choices=['true', 'false'],
            description='Transferir la pinza al bringup incluso en modo fake. Sólo para '
                        'diagnosticar la incompatibilidad de mimic joints de ros2_kortex '
                        'con Jazzy: aborta el ros2_control_node.'),
        DeclareLaunchArgument(
            'robot_controller',
            default_value=str(defaults.get('robot_controller', 'joint_trajectory_controller')),
            description='Controlador de trayectoria cargado por el bringup.'),
    ]

    return LaunchDescription(declared + [OpaqueFunction(function=_setup)])
