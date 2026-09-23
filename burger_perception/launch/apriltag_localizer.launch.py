r"""
Localizador AprilTag de burger_perception.

    ros2 launch burger_perception apriltag_localizer.launch.py                # simulado
    ros2 launch burger_perception apriltag_localizer.launch.py \\
        params_file:=<repo>/vision_setup/tags_fisicos.yaml publish_tf:=true   # cámara real
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('burger_perception'), 'config', 'apriltag_localizer.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('publish_tf', default_value='false',
                              description='Publicar TF tag_mesa -> tag_carrito1'),
        Node(
            package='burger_perception',
            executable='apriltag_localizer',
            name='apriltag_fixed_camera_localizer',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool)},
            ]),
    ])
