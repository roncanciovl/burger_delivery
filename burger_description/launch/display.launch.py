import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_static_carts = LaunchConfiguration('use_static_carts', default='false')
    urdf_file_name = 'delivery_scene_fixed.urdf'

    urdf = os.path.join(
        get_package_share_directory('burger_description'),
        'urdf',
        urdf_file_name)
    car1_urdf = os.path.join(
        get_package_share_directory('burger_description'),
        'urdf',
        'car1_apriltag.urdf')
    car2_urdf = os.path.join(
        get_package_share_directory('burger_description'),
        'urdf',
        'car2_apriltag.urdf')

    rviz_config_file = os.path.join(
        get_package_share_directory('burger_description'),
        'rviz',
        'default.rviz')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    with open(car1_urdf, 'r') as infp:
        car1_desc = infp.read()
    with open(car2_urdf, 'r') as infp:
        car2_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'use_static_carts',
            default_value='false',
            description='Publica TFs temporales tag_mesa -> tag_carrito{1,2} mientras no exista el nodo de localizacion AprilTag'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='car1_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': car1_desc}],
            remappings=[
                ('robot_description', '/car1/robot_description'),
                ('joint_states', '/car1/joint_states')],
            arguments=[car1_urdf]),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='car1_joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('joint_states', '/car1/joint_states')],
            arguments=[car1_urdf]),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='car2_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': car2_desc}],
            remappings=[
                ('robot_description', '/car2/robot_description'),
                ('joint_states', '/car2/joint_states')],
            arguments=[car2_urdf]),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='car2_joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('joint_states', '/car2/joint_states')],
            arguments=[car2_urdf]),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.35', '--y', '0.10', '--z', '0.00',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'tag_mesa',
                '--child-frame-id', 'tag_carrito1'],
            condition=IfCondition(use_static_carts)),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.35', '--y', '-0.10', '--z', '0.00',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'tag_mesa',
                '--child-frame-id', 'tag_carrito2'],
            condition=IfCondition(use_static_carts)),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]),
    ])
