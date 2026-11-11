r"""
Cámara del Kinova conectada al brazo, con la calibración del laboratorio. Sólo la anfitriona.

    ros2 launch burger_perception camera.launch.py                      # calibración medida
    ros2 launch burger_perception camera.launch.py calibracion:=kinova  # valores de fábrica

Lanza los dos nodos de ``kinova_vision`` (color y profundidad), el registro de la profundidad en el
marco del color y tres transformaciones fijas:

* ``camera_link -> camera_color_frame``: identidad (el color define ``camera_link``).
* ``camera_link -> camera_depth_frame``: los extrínsecos de
  ``config/camara_kinova/extrinsecos_profundidad_color.yaml`` (hoy, los de fábrica: con la
  calibración nueva del color son los que mejor validan).
* ``end_effector_link -> camera_link``: el eslabón que el driver no publica porque carga el brazo
  sin ``vision:=true`` (TALLER_URDF_TF.md §11). Valores del macro oficial de Kinova.

La calibración del color sale de ``config/camara_kinova/color_1920x1080.yaml``; sólo vale para el
stream de 1920x1080. La de fábrica (``cy`` = 97) dejaba la profundidad registrada unos 3-4 cm fuera
del color. Origen, método y validación: ``config/camara_kinova/README.md``.

No se reutiliza ``kinova_vision.launch.py`` porque fija sus extrínsecos sin argumento para
cambiarlos, y este launch los lee de un archivo. Si otro proceso ya publica el eslabón
brazo-cámara, use ``publish_arm_tf:=false``.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import yaml

KINOVA_DEPTH_T = (-0.0195, -0.005, 0.0)


def _vision_node(kind, device, calibration_url):
    rtsp = f'rtspsrc location=rtsp://{device}/{kind} latency=30'
    pipeline = (f'{rtsp} ! rtph264depay ! avdec_h264 ! videoconvert' if kind == 'color'
                else f'{rtsp} ! rtpgstdepay')
    prefix = 'color' if kind == 'color' else 'depth'
    return Node(
        package='kinova_vision', executable='kinova_vision_node', namespace='camera',
        name=f'kinova_vision_{kind}', output='both',
        parameters=[{
            'camera_type': kind, 'camera_name': kind,
            'camera_info_url_default':
                f'package://kinova_vision/launch/calibration/default_{kind}_calib_%ux%u.ini',
            'camera_info_url_user': calibration_url,
            'stream_config': pipeline, 'frame_id': f'camera_{kind}_frame', 'max_pub_rate': 30.0,
        }],
        remappings=[(topic, f'{prefix}/{topic}') for topic in (
            'camera_info', 'image_raw', 'image_raw/compressed', 'image_raw/compressedDepth',
            'image_raw/theora')])


def _static_tf(name, parent, child, xyz, rpy):
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=name, output='screen',
        arguments=['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                   '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                   '--frame-id', parent, '--child-frame-id', child])


def _setup(context):
    config = os.path.join(get_package_share_directory('burger_perception'), 'config',
                          'camara_kinova')
    device = LaunchConfiguration('device').perform(context)
    calibration = LaunchConfiguration('calibracion').perform(context)
    if calibration == 'laboratorio':
        color_url = 'file://' + os.path.join(config, 'color_1920x1080.yaml')
        path = os.path.join(config, 'extrinsecos_profundidad_color.yaml')
        with open(path, encoding='utf-8') as f:
            extrinsics = yaml.safe_load(f)
        depth_xyz, depth_rpy = extrinsics['translation'], extrinsics['rpy_rad']
    elif calibration == 'kinova':
        color_url, depth_xyz, depth_rpy = '', KINOVA_DEPTH_T, (0.0, 0.0, 0.0)
    else:
        raise RuntimeError(f'calibracion:={calibration} no válida (laboratorio | kinova)')

    register = ComposableNodeContainer(
        name='registered_depth_images', namespace='camera', package='rclcpp_components',
        executable='component_container', output='both',
        composable_node_descriptions=[ComposableNode(
            package='depth_image_proc', plugin='depth_image_proc::RegisterNode',
            name='register_node', namespace='camera',
            remappings=[('rgb/camera_info', 'color/camera_info'),
                        ('depth/camera_info', 'depth/camera_info'),
                        ('depth/image_rect', 'depth/image_raw')],
            parameters=[{'fill_upsampling_holes': True}])],
        condition=IfCondition(LaunchConfiguration('depth_registration')))

    return [
        _vision_node('color', device, color_url),
        _vision_node('depth', device, ''),
        _static_tf('camera_color_tf', 'camera_link', 'camera_color_frame', (0, 0, 0), (0, 0, 0)),
        _static_tf('camera_depth_tf', 'camera_link', 'camera_depth_frame', depth_xyz, depth_rpy),
        register,
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='camera_arm_tf_publisher', output='screen',
             arguments=['--x', '0', '--y', '0.05639', '--z', '-0.00305',
                        '--roll', str(math.pi), '--pitch', str(math.pi), '--yaw', '0',
                        '--frame-id', 'end_effector_link', '--child-frame-id', 'camera_link'],
             condition=IfCondition(LaunchConfiguration('publish_arm_tf'))),
    ]


def generate_launch_description():
    """Crear la descripción: cámara calibrada, registro y eslabón brazo-cámara."""
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='192.168.1.10', description='IP del Kinova'),
        DeclareLaunchArgument('calibracion', default_value='laboratorio',
                              description='laboratorio (medida) o kinova (valores de fábrica)'),
        DeclareLaunchArgument('depth_registration', default_value='true',
                              description='Publicar /camera/depth_registered/image_rect'),
        DeclareLaunchArgument('publish_arm_tf', default_value='true',
                              description='Publicar end_effector_link -> camera_link'),
        OpaqueFunction(function=_setup),
    ])
