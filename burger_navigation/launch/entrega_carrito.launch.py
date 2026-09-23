r"""
Acople de TF y acción de entrega para UN carrito.

    ros2 launch burger_navigation entrega_carrito.launch.py car:=car1 \
        pose_topic:=/burger_car_01/pose2d use_nav2:=false
    ros2 action send_goal /car1/prepare_delivery_pose \
        burger_interfaces/action/PrepareDeliveryPose "{slot_id: slot_entrega_1}" --feedback

Con use_nav2:=false el servidor no navega: el carrito llega por su cuenta (o a mano) y la
acción sólo confirma con el AprilTag que está dentro de la tolerancia.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    del args, kwargs

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    car = arg('car')
    numero = ''.join(c for c in car if c.isdigit()) or '1'
    base = arg('base_frame') or f'car{numero}_base_link'
    slots = arg('slots_file') or os.path.join(
        get_package_share_directory('burger_navigation'), 'config', 'delivery_slots.yaml')
    return [
        Node(package='burger_navigation', executable='car_tf_coupler',
             name='car_tf_coupler', namespace=car, output='screen',
             parameters=[{'pose_topic': arg('pose_topic'),
                          'tag_frame': f'tag_carrito{numero}',
                          'base_frame': base,
                          'odom_frame': arg('odom_frame')}]),
        Node(package='burger_navigation', executable='delivery_pose_server',
             name='delivery_pose_server', namespace=car, output='screen',
             parameters=[{'slots_file': slots, 'car_base_frame': base,
                          'use_nav2': arg('use_nav2').lower() == 'true'}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('car', default_value='car1'),
        DeclareLaunchArgument('pose_topic', default_value='/burger_car_01/pose2d'),
        DeclareLaunchArgument('base_frame', default_value=''),
        DeclareLaunchArgument('odom_frame', default_value=''),
        DeclareLaunchArgument('slots_file', default_value=''),
        DeclareLaunchArgument('use_nav2', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
