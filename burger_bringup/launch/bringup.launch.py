r"""
Arranque centralizado de Burger-Cell (TODO.md §2, burger_bringup).

Modos:

* ``simulation:=true`` (por defecto): escena de burger_description en RViz y localizador
  AprilTag simulado. No toca ningún robot.
* ``simulation:=false``: conexión con el Kinova (burger_kinova_reference) y localizador con
  la cámara real. Sólo la estación anfitriona usa ``start_driver:=true``
  (TROUBLESHOOTING.md §2.0); las demás se conectan al driver que ya corre.

Módulos opcionales: ``use_apriltag`` (true), ``use_vlm`` (false), ``use_moveit`` (false,
sólo planifica) y ``use_telemetry`` (false).

    ros2 launch burger_bringup bringup.launch.py
    ros2 launch burger_bringup bringup.launch.py simulation:=false robot_ip:=192.168.1.10 \
        start_driver:=true use_fake_hardware:=false \
        apriltag_params:=<repo>/vision_setup/tags_fisicos.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _share(package, *path):
    return os.path.join(get_package_share_directory(package), *path)


def _include(package, launch_file, **arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_share(package, 'launch', launch_file)),
        launch_arguments={k: str(v) for k, v in arguments.items()}.items())


def _setup(context, *args, **kwargs):
    del args, kwargs

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return arg(name).lower() in ('true', '1', 'yes')

    simulation = flag('simulation')
    modulos = {nombre: flag(f'use_{nombre}')
               for nombre in ('apriltag', 'vlm', 'moveit', 'telemetry')}
    resumen = ', '.join(f'{nombre}={valor}' for nombre, valor in modulos.items())
    acciones = [LogInfo(msg=f'burger_bringup: simulation={simulation}, {resumen}')]

    if simulation:
        acciones.append(_include('burger_description', 'display.launch.py',
                                 use_static_carts='false' if flag('use_apriltag') else 'true'))
    else:
        acciones.append(_include(
            'burger_kinova_reference', 'kinova_connection.launch.py',
            start_driver=arg('start_driver'), robot_ip=arg('robot_ip'),
            use_fake_hardware=arg('use_fake_hardware'), launch_rviz=arg('launch_rviz')))

    if flag('use_apriltag'):
        params = arg('apriltag_params') or _share(
            'burger_perception', 'config', 'apriltag_localizer.yaml')
        acciones.append(_include('burger_perception', 'apriltag_localizer.launch.py',
                                 params_file=params, publish_tf='true'))
        if not simulation and not arg('apriltag_params'):
            acciones.append(LogInfo(msg='⚠ simulation:=false sin apriltag_params: el '
                                        'localizador usa la configuración simulada. Pasa '
                                        'apriltag_params:=<repo>/vision_setup/tags_fisicos.yaml'))

    if flag('use_vlm'):
        acciones.append(LogInfo(msg='⚠ use_vlm:=true: el nodo Gemini aún no está en este '
                                    'branch (ver claude/todo-gemini-vlm-node)'))

    if flag('use_moveit'):
        acciones.append(_include('burger_control', 'pick_place.launch.py',
                                 robot_ip=arg('robot_ip'),
                                 use_fake_hardware=arg('use_fake_hardware'),
                                 execute='false'))

    if flag('use_telemetry'):
        acciones.append(_include('burger_telemetry', 'telemetry.launch.py',
                                 monitor='true', record='true'))
    return acciones


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='true'),
        DeclareLaunchArgument('use_apriltag', default_value='true'),
        DeclareLaunchArgument('apriltag_params', default_value=''),
        DeclareLaunchArgument('use_vlm', default_value='false'),
        DeclareLaunchArgument('use_moveit', default_value='false'),
        DeclareLaunchArgument('use_telemetry', default_value='false'),
        DeclareLaunchArgument('start_driver', default_value='false'),
        DeclareLaunchArgument('robot_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('use_fake_hardware', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
