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
Lanzar la cámara de color del Kinova publicando sólo imagen comprimida.

Uso normal::

    ros2 launch burger_kinova_reference kinova_vision_compressed.launch.py

El nodo conserva ``camera_info`` porque contiene la calibración, pero limita
los publicadores de ``image_transport`` al plugin ``compressed``. Por tanto no
anuncia ``/camera/color/image_raw``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _launch_camera(context):
    """Construir el nodo después de resolver los argumentos del launch."""
    device = LaunchConfiguration('device').perform(context)
    latency = LaunchConfiguration('rtsp_latency_ms').perform(context)
    namespace = LaunchConfiguration('camera_namespace').perform(context)

    stream_config = (
        f'rtspsrc location=rtsp://{device}/color latency={latency} '
        '! rtph264depay ! avdec_h264 ! videoconvert'
    )

    return [
        LogInfo(msg=(
            'Kinova Vision: se publicará únicamente '
            f'/{namespace.strip("/")}/color/image_raw/compressed'
        )),
        Node(
            package='kinova_vision',
            executable='kinova_vision_node',
            namespace=namespace,
            name='kinova_vision_color',
            output='screen',
            remappings=[
                ('camera_info', 'color/camera_info'),
                ('image_raw', 'color/image_raw'),
                ('image_raw/compressed', 'color/image_raw/compressed'),
            ],
            parameters=[{
                'camera_type': 'color',
                'camera_name': 'color',
                'frame_id': LaunchConfiguration('color_frame_id'),
                'max_pub_rate': float(
                    LaunchConfiguration('max_pub_rate').perform(context)
                ),
                'camera_info_url_user': '',
                'camera_info_url_default': (
                    'package://kinova_vision/launch/calibration/'
                    'default_color_calib_%ux%u.ini'
                ),
                'stream_config': stream_config,
                'image_raw.enable_pub_plugins': [
                    'image_transport/compressed',
                ],
            }],
        ),
    ]


def generate_launch_description():
    """Crear la descripción del launch de visión comprimida."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='192.168.1.10',
            description='IP o nombre de red del Kinova.',
        ),
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='camera',
            description='Namespace ROS 2 de la cámara.',
        ),
        DeclareLaunchArgument(
            'color_frame_id',
            default_value='camera_color_frame',
            description='Frame de la imagen de color.',
        ),
        DeclareLaunchArgument(
            'max_pub_rate',
            default_value='30.0',
            description='Frecuencia máxima de publicación en Hz.',
        ),
        DeclareLaunchArgument(
            'rtsp_latency_ms',
            default_value='30',
            description='Latencia del receptor RTSP en milisegundos.',
        ),
        OpaqueFunction(function=_launch_camera),
    ])
