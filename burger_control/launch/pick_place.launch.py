r"""
Pick & place de la caja con MoveIt Task Constructor sobre el Gen3 6 GDL + Robotiq 2F-85.

Levanta move_group con la capacidad ExecuteTaskSolution (la que necesita MTC para
ejecutar), el nodo de la tarea y, si se pide, RViz. NO lanza el driver: con el robot real
el driver lo tiene la anfitriona (TROUBLESHOOTING.md §2.0); sin robot, lanza antes el
driver en modo fake:

    ros2 launch kortex_bringup gen3.launch.py dof:=6 gripper:=robotiq_2f_85 \
        robot_ip:=0.0.0.0 use_fake_hardware:=true launch_rviz:=false

Luego:

    ros2 launch burger_control pick_place.launch.py                    # sólo planifica
    ros2 launch burger_control pick_place.launch.py execute:=true      # ejecuta

No combinar con robot.launch.py de kinova_gen3_6dof_robotiq_2f_85_moveit_config: ese
launch trae su propio ros2_control_node y su propio move_group.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MOVEIT_CONFIG = 'kinova_gen3_6dof_robotiq_2f_85_moveit_config'


def _setup(context, *args, **kwargs):
    del args, kwargs
    from moveit_configs_utils import MoveItConfigsBuilder

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    mappings = {
        'robot_ip': arg('robot_ip'),
        'use_fake_hardware': arg('use_fake_hardware'),
        'gripper': 'robotiq_2f_85',
        'gripper_joint_name': 'robotiq_85_left_knuckle_joint',
        'dof': '6',
    }
    moveit_config = (
        MoveItConfigsBuilder('gen3', package_name=MOVEIT_CONFIG)
        .robot_description(mappings=mappings)
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    params_file = os.path.join(
        get_package_share_directory('burger_control'), 'config', 'pick_place.yaml')
    execute = arg('execute').lower() in ('true', '1', 'yes')

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(),
                    {'capabilities': 'move_group/ExecuteTaskSolutionCapability'}],
        condition=IfCondition(LaunchConfiguration('start_move_group')),
    )
    task = Node(
        package='burger_control',
        executable='pick_place_node',
        name='burger_pick_place',
        output='screen',
        parameters=[moveit_config.to_dict(), params_file, {'execute': execute}],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', os.path.join(
            get_package_share_directory(MOVEIT_CONFIG), 'config', 'moveit.rviz')],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )
    return [move_group, task, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('use_fake_hardware', default_value='true'),
        DeclareLaunchArgument('execute', default_value='false',
                              description='true = ejecutar en el robot; false = sólo planificar'),
        DeclareLaunchArgument('start_move_group', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
