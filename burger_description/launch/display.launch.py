"""
Visor de la escena Burger-Cell: robot_state_publisher, joint_state_publisher_gui y RViz.

Es un visor SIN robot: publica /joint_states, /tf y /robot_description propios. En el
dominio del curso (ROS_DOMAIN_ID=0, el mismo del driver del Kinova) esos nombres chocan
con los del robot real y con los visores de las demás estaciones, sin ningún error
(TROUBLESHOOTING.md §4.1). Dos maneras de evitarlo:

* un dominio propio por estación (lo que piden los talleres), o
* ``namespace:=<nombre>``: todo, incluidos /tf y /tf_static, queda bajo /<nombre>/.

Ejemplos::

    ros2 launch burger_description display.launch.py use_static_carts:=true
    ros2 launch burger_description display.launch.py namespace:=visor_est07
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'burger_description'


def _read(path):
    with open(path, 'r') as infp:
        return infp.read()


def isolation_warning(namespace, domain_id):
    """
    Devolver el aviso de aislamiento que corresponde, o una cadena vacía.

    :param namespace: namespace pedido por argumento ('' = sin namespace).
    :param domain_id: valor de ROS_DOMAIN_ID ('' si no está definido).
    """
    if namespace:
        return ''
    domain = domain_id.strip() or '0'
    if domain != '0':
        return ''
    return (
        '⚠ display.launch.py publica /joint_states, /tf y /robot_description SIN '
        'namespace en el dominio 0, el mismo del driver del Kinova. Si hay un robot o '
        'visores de otras estaciones en este dominio, sus datos se mezclan en silencio. '
        'Usa un dominio propio (export ROS_DOMAIN_ID=<n>) o relanza con '
        'namespace:=<nombre>. Ver TROUBLESHOOTING.md §4.1.')


def _setup(context, *args, **kwargs):
    del args, kwargs
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_static_carts = LaunchConfiguration('use_static_carts')

    share = get_package_share_directory(PACKAGE)
    urdf = os.path.join(share, 'urdf', 'delivery_scene_fixed.urdf')
    car1_urdf = os.path.join(share, 'urdf', 'car1_apriltag.urdf')
    car2_urdf = os.path.join(share, 'urdf', 'car2_apriltag.urdf')
    rviz_config_file = os.path.join(share, 'rviz', 'default.rviz')

    prefix = f'/{namespace}' if namespace else ''
    # /tf y /tf_static son absolutos en tf2: sin este remapeo seguirían siendo globales
    # aunque el resto de tópicos quede bajo el namespace.
    tf_remaps = [('/tf', f'{prefix}/tf'), ('/tf_static', f'{prefix}/tf_static')]

    def car_nodes(car, car_urdf_path):
        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name=f'{car}_state_publisher',
                namespace=namespace,
                output='screen',
                parameters=[{'use_sim_time': use_sim_time,
                             'robot_description': _read(car_urdf_path)}],
                remappings=[
                    ('robot_description', f'{car}/robot_description'),
                    ('joint_states', f'{car}/joint_states')] + tf_remaps,
                arguments=[car_urdf_path]),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name=f'{car}_joint_state_publisher',
                namespace=namespace,
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                remappings=[('joint_states', f'{car}/joint_states')],
                arguments=[car_urdf_path]),
        ]

    def static_cart(child, y):
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=namespace,
            remappings=tf_remaps,
            arguments=[
                '--x', '0.35', '--y', y, '--z', '0.00',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'tag_mesa',
                '--child-frame-id', child],
            condition=IfCondition(use_static_carts))

    actions = []
    warning = isolation_warning(namespace, os.environ.get('ROS_DOMAIN_ID', ''))
    if warning:
        actions.append(LogInfo(msg=warning))
    else:
        actions.append(LogInfo(msg=(
            f'Visor aislado: namespace "{prefix or "/"}", '
            f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}')))

    actions += [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': _read(urdf)}],
            remappings=tf_remaps,
            arguments=[urdf]),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),
        *car_nodes('car1', car1_urdf),
        *car_nodes('car2', car2_urdf),
        static_cart('tag_carrito1', '0.10'),
        static_cart('tag_carrito2', '-0.10'),
        # default.rviz nombra los tópicos en absoluto (/robot_description, ...): se
        # remapean al namespace para no tener que mantener una configuración por visor.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            output='screen',
            remappings=tf_remaps + [
                ('/robot_description', f'{prefix}/robot_description'),
                ('/car1/robot_description', f'{prefix}/car1/robot_description'),
                ('/car2/robot_description', f'{prefix}/car2/robot_description'),
            ] if namespace else [],
            arguments=['-d', rviz_config_file]),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_static_carts',
            default_value='false',
            description='Publica TFs temporales tag_mesa -> tag_carrito{1,2} mientras no '
                        'exista el nodo de localizacion AprilTag'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace del visor. Vacío = nombres globales (/tf, '
                        '/joint_states, /robot_description), como antes'),
        OpaqueFunction(function=_setup),
    ])
