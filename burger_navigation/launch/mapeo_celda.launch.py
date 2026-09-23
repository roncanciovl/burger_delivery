r"""
Mapeo de la celda con slam_toolbox (TurtleBot3 u otro AGV con LiDAR 2D).

    ros2 launch burger_navigation mapeo_celda.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard     # recorrer la celda despacio
    ros2 run nav2_map_server map_saver_cli -f ~/mapas/celda   # guardar al terminar

Los carritos ESP32 sin LiDAR no pueden mapear: para ellos el mapa no es necesario porque
se localizan con AprilTag (ver docs/navigation/NAV2_ENTREGAS.md).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('burger_navigation'), 'config',
                          'slam_toolbox_celda.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
