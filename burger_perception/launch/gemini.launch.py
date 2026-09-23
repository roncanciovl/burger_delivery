r"""
Nodo de razonamiento espacial con Gemini Robotics-ER.

    export GEMINI_API_KEY=...
    ros2 launch burger_perception gemini.launch.py
    ros2 service call /gemini_spatial_reasoning/locate std_srvs/srv/Trigger
    ros2 run tf2_ros tf2_echo base_link target_burger_box_frame
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('burger_perception'), 'config', 'gemini.yaml')
    acciones = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='burger_perception',
            executable='gemini_spatial_reasoning_node',
            name='gemini_spatial_reasoning',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]),
    ]
    if not (os.environ.get('GEMINI_API_KEY') or os.environ.get('GOOGLE_API_KEY')):
        acciones.insert(0, LogInfo(msg='⚠ GEMINI_API_KEY no está definida en esta terminal: '
                                       'el nodo no arrancará'))
    return LaunchDescription(acciones)
