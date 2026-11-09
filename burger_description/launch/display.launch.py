import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_static_carts = LaunchConfiguration('use_static_carts', default='false')
    robot_real = LaunchConfiguration('robot_real', default='false')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default='false')
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
    rviz_lab_file = os.path.join(
        get_package_share_directory('burger_description'),
        'rviz',
        'laboratorio.rviz')

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

        # robot_real:=true muestra la escena con la pose real del Kinova: en vez del
        # deslizador, scene_joint_relay copia /joint_states del driver a /scene/joint_states
        # con el prefijo gen3_. La escena publica en /scene/robot_description para no
        # pisar el /robot_description del driver.
        DeclareLaunchArgument(
            'robot_real',
            default_value='false',
            description='Mover el brazo con /joint_states del driver del Kinova en lugar del deslizador'),

        # El driver fake no lanza la pinza: sin esto, RViz marca sus eslabones sin TF.
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Con robot_real:=true, el driver es fake: publicar la pinza abierta en la escena'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf],
            condition=UnlessCondition(robot_real)),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf],
            condition=UnlessCondition(robot_real)),

        Node(
            package='burger_description',
            executable='scene_joint_relay.py',
            name='scene_joint_relay',
            output='screen',
            parameters=[{'fill_gripper': ParameterValue(use_fake_hardware, value_type=bool)}],
            condition=IfCondition(robot_real)),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='scene',
            name='scene_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            condition=IfCondition(robot_real)),

        Node(
            package='burger_description',
            executable='scene_collision_markers.py',
            name='scene_collision_markers',
            output='screen',
            parameters=[{'scene_urdf': urdf}]),

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
                '--x', '0.35', '--y', '0.18', '--z', '0.168',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'tag_mesa',
                '--child-frame-id', 'tag_carrito1'],
            condition=IfCondition(use_static_carts)),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.35', '--y', '-0.18', '--z', '0.213',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'tag_mesa',
                '--child-frame-id', 'tag_carrito2'],
            condition=IfCondition(use_static_carts)),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            condition=UnlessCondition(robot_real)),

        # Con el robot real, RViz comparte la CPU con el ciclo de control del driver, que en
        # WSL no obtiene prioridad de tiempo real. Con prioridad baja, RViz deja de retrasar
        # ese ciclo (ver TROUBLESHOOTING.md §3.7).
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_laboratorio',
            output='screen',
            prefix='nice -n 19',
            arguments=['-d', rviz_lab_file],
            condition=IfCondition(robot_real)),
    ])
