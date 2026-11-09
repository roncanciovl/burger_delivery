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

"""
Visualizar en RViz las extrínsecas de los sensores RGB y de profundidad.

Por defecto el launch sólo abre RViz y consume el árbol TF existente. Para una
demostración sin robot, ``demo_tf:=true`` publica las extrínsecas nominales del
launch oficial de ``kinova_vision``.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Crear la demostración de frames de la cámara Kinova."""
    package_share = get_package_share_directory('burger_kinova_reference')
    rviz_config = os.path.join(
        package_share,
        'rviz',
        'kinova_camera_frames.rviz',
    )
    demo_tf = LaunchConfiguration('demo_tf')

    # El prefijo separa el modelo descriptivo de los frames cuya extrínseca se
    # demuestra. Así no existen dos autoridades para camera_{color,depth}_frame.
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('kortex_description'),
            'robots',
            'kinova.urdf.xacro',
        ]),
        ' name:=kinova_camera_demo',
        ' arm:=gen3',
        ' dof:=6',
        ' vision:=true',
        ' prefix:=demo_',
        ' robot_ip:=0.0.0.0',
        ' use_fake_hardware:=true',
    ])
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content,
            value_type=str,
        ),
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='demo_kinova_state_publisher',
        parameters=[robot_description],
        condition=IfCondition(demo_tf),
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='demo_kinova_joint_state_publisher',
        condition=IfCondition(demo_tf),
        output='screen',
    )

    model_camera_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='demo_model_camera_bridge',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'demo_camera_link',
            '--child-frame-id', 'camera_link',
        ],
        condition=IfCondition(demo_tf),
        output='screen',
    )

    color_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='demo_camera_color_tf_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_color_frame',
        ],
        condition=IfCondition(demo_tf),
        output='screen',
    )

    depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='demo_camera_depth_tf_publisher',
        arguments=[
            '--x', '-0.0195', '--y', '-0.005', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_depth_frame',
        ],
        condition=IfCondition(demo_tf),
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='kinova_camera_frames_rviz',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'demo_tf',
            default_value='false',
            description=(
                'Publicar extrínsecas nominales para una demostración '
                'sin robot. '
                'Debe ser false si otro nodo ya publica estos frames.'
            ),
        ),
        LogInfo(
            condition=IfCondition(demo_tf),
            msg=(
                'MODO DEMO: extrínsecas nominales; no se abre RTSP ni se '
                'publican imágenes.'
            ),
        ),
        LogInfo(
            condition=UnlessCondition(demo_tf),
            msg='MODO REAL: RViz espera los frames publicados en /tf_static.',
        ),
        robot_state_publisher,
        joint_state_publisher,
        model_camera_bridge,
        color_tf,
        depth_tf,
        rviz,
    ])
