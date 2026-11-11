r"""
Telemetría de Burger-Cell en un solo launch.

* ``monitor:=true``  monitor de red web (network_setup/iniciar_monitor.sh, puerto 8080).
* ``microros:=true`` agente micro-ROS por UDP en el puerto 8888 (ESP32 de los carritos).
* ``record:=true``   bolsa MCAP con compresión zstd de los tópicos de telemetría, con las
  reglas del taller de rosbag2 (lista explícita de tópicos, nunca imágenes crudas).

El monitor vive en el repositorio y no en install/: se ubica con ``repo_path`` (por
defecto $BURGER_REPO o ~/ros2_ws/src/burger_delivery).

    ros2 launch burger_telemetry telemetry.launch.py monitor:=true record:=true
"""

import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TOPICOS = [
    '/joint_states',
    '/burger/kinova/diagnostics',
    '/rosout',
    '/tf',
    '/tf_static',
    '/burger_car_01/pose2d',
    '/burger_car_02/pose2d',
]


def _true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def _setup(context, *args, **kwargs):
    del args, kwargs
    acciones = []
    repo = LaunchConfiguration('repo_path').perform(context)
    if _true(context, 'monitor'):
        script = os.path.join(repo, 'network_setup', 'iniciar_monitor.sh')
        if os.path.isfile(script):
            acciones.append(ExecuteProcess(
                cmd=['bash', script, LaunchConfiguration('monitor_port').perform(context)],
                output='screen'))
        else:
            acciones.append(LogInfo(msg=f'⚠ No encuentro {script}; indica repo_path:=<repo>'))
    if _true(context, 'microros'):
        acciones.append(Node(
            package='micro_ros_agent', executable='micro_ros_agent', output='screen',
            arguments=['udp4', '--port', LaunchConfiguration('microros_port').perform(context)]))
    if _true(context, 'record'):
        salida = LaunchConfiguration('bag_output').perform(context) or \
            time.strftime('telemetria_burger_%Y%m%d_%H%M%S')
        acciones.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--storage', 'mcap',
                 '--compression-mode', 'file', '--compression-format', 'zstd',
                 '--max-bag-duration', '60', '--output', salida, '--topics', *TOPICOS],
            output='screen'))
    if not acciones:
        acciones.append(LogInfo(msg='burger_telemetry: nada que lanzar (monitor, microros y '
                                    'record están en false)'))
    return acciones


def generate_launch_description():
    repo_defecto = os.environ.get(
        'BURGER_REPO', os.path.expanduser('~/ros2_ws/src/burger_delivery'))
    return LaunchDescription([
        DeclareLaunchArgument('monitor', default_value='true'),
        DeclareLaunchArgument('monitor_port', default_value='8080'),
        DeclareLaunchArgument('microros', default_value='false'),
        DeclareLaunchArgument('microros_port', default_value='8888'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_output', default_value=''),
        DeclareLaunchArgument('repo_path', default_value=repo_defecto),
        OpaqueFunction(function=_setup),
    ])
