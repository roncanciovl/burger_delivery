r"""
Nav2 para un carrito de la celda, con mapa de SLAM.

Levanta map_server, planner, controller, behaviors, bt_navigator y velocity_smoother bajo
el namespace del carrito. Los parámetros son los de nav2_bringup más los ajustes de
config/nav2_carrito.yaml (sólo diferencias), con los marcos del carrito reescritos.

    ros2 launch burger_navigation navegacion_celda.launch.py car:=car1 \
        map:=$HOME/mapas/celda.yaml base_frame:=base_footprint odom_frame:=odom
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

# (paquete, nodo); el ejecutable se llama igual que el nodo.
NODOS = [('nav2_map_server', 'map_server'), ('nav2_planner', 'planner_server'),
         ('nav2_controller', 'controller_server'), ('nav2_behaviors', 'behavior_server'),
         ('nav2_bt_navigator', 'bt_navigator'), ('nav2_velocity_smoother', 'velocity_smoother')]


def _setup(context, *args, **kwargs):
    del args, kwargs
    from nav2_common.launch import RewrittenYaml

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    car = arg('car')
    reescrituras = {
        'robot_base_frame': arg('base_frame'),
        'global_frame': 'map',
        'odom_topic': f'/{car}/odom',
        'yaml_filename': arg('map'),
        'use_sim_time': arg('use_sim_time'),
    }
    base = os.path.join(get_package_share_directory('nav2_bringup'), 'params',
                        'nav2_params.yaml')
    propios = os.path.join(get_package_share_directory('burger_navigation'), 'config',
                           'nav2_carrito.yaml')
    params = [RewrittenYaml(source_file=f, root_key=car, param_rewrites=reescrituras,
                            convert_types=True) for f in (base, propios)]
    # El costmap local sigue a la odometría del carrito; sin odometría, al mapa. Es un
    # subnodo (/<car>/local_costmap/local_costmap): se le pasa con un YAML propio.
    local = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    yaml.safe_dump({car: {'local_costmap': {'local_costmap': {'ros__parameters': {
        'global_frame': arg('odom_frame') or 'map'}}}}}, local)
    local.close()
    params.append(local.name)

    acciones = []
    for paquete, nodo in NODOS:
        remaps = []
        if nodo == 'controller_server':
            remaps = [('cmd_vel', 'cmd_vel_nav')]
        if nodo == 'velocity_smoother':
            remaps = [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
        acciones.append(Node(package=paquete, executable=nodo, name=nodo,
                             namespace=car, output='screen', parameters=params,
                             remappings=remaps))
    acciones.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', namespace=car, output='screen',
        parameters=[{'autostart': True, 'node_names': [nodo for _, nodo in NODOS],
                     'use_sim_time': arg('use_sim_time') == 'true'}]))
    return acciones


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('car', default_value='car1'),
        DeclareLaunchArgument('map', description='mapa .yaml guardado con map_saver_cli'),
        DeclareLaunchArgument('base_frame', default_value='car1_base_link'),
        DeclareLaunchArgument('odom_frame', default_value='',
                              description='marco de odometría del carrito; vacío = sin odom'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
