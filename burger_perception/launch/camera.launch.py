r"""
Cámara del Kinova conectada al brazo. Sólo la lanza la estación anfitriona.

    ros2 launch burger_perception camera.launch.py

Lanza ``kinova_vision`` con profundidad registrada y publica el eslabón fijo
``end_effector_link -> camera_link``, que el driver no publica porque carga el brazo sin
``vision:=true`` (TALLER_URDF_TF.md §11). Los valores son los del macro oficial de Kinova.

Si otro proceso ya publica ese eslabón (por ejemplo, un driver con ``vision:=true``), use
``publish_arm_tf:=false``: cada frame admite un solo padre.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Crear la descripción: cámara con registro y eslabón brazo-cámara."""
    vision = os.path.join(
        get_package_share_directory('kinova_vision'), 'launch', 'kinova_vision.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='192.168.1.10',
                              description='IP del Kinova'),
        DeclareLaunchArgument('publish_arm_tf', default_value='true',
                              description='Publicar end_effector_link -> camera_link'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vision),
            launch_arguments={'device': LaunchConfiguration('device'),
                              'depth_registration': 'true'}.items()),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_arm_tf_publisher',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0.05639', '--z', '-0.00305',
                '--roll', str(math.pi), '--pitch', str(math.pi), '--yaw', '0',
                '--frame-id', 'end_effector_link', '--child-frame-id', 'camera_link',
            ],
            condition=IfCondition(LaunchConfiguration('publish_arm_tf'))),
    ])
