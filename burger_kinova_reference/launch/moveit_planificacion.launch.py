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

r"""
MoveIt 2 en modo SÓLO PLANIFICACIÓN para el Kinova Gen3 6 GDL + Robotiq 2F-85.

Levanta:

* ``move_group`` con ``allow_trajectory_execution: false``: planifica, pero no puede
  mover el robot (tampoco con el botón *Execute* de RViz). La única puerta hacia el
  controlador es ``moveit_safe_executor``.
* RViz con el panel MotionPlanning (``launch_rviz:=true``).
* ``escena_moveit``: carga la mesa de ``burger_description`` como obstáculo
  (``cargar_escena:=true``).

NO lanza el driver. Con el robot real, el driver lo tiene la estación anfitriona
(``kinova_connection.launch.py start_driver:=true``, TROUBLESHOOTING.md §2.0). Sin robot,
lánzalo antes en modo fake::

    ros2 launch burger_kinova_reference kinova_connection.launch.py \
        start_driver:=true robot_ip:=0.0.0.0 use_fake_hardware:=true
    ros2 launch burger_kinova_reference moveit_planificacion.launch.py

No usar ``robot.launch.py`` de ``kinova_gen3_6dof_robotiq_2f_85_moveit_config``: ese
launch arranca su propio ``ros2_control_node``, es decir, un segundo driver.

Después, en otra terminal (la confirmación por teclado necesita una terminal propia)::

    SHARE=$(ros2 pkg prefix burger_kinova_reference)/share/burger_kinova_reference
    ros2 run burger_kinova_reference moveit_safe_executor --ros-args \
        --params-file $SHARE/config/kinova_connection.yaml \
        -p goals_file:=$SHARE/config/metas_moveit.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

PACKAGE_NAME = 'burger_kinova_reference'
MOVEIT_CONFIG = 'kinova_gen3_6dof_robotiq_2f_85_moveit_config'


def _as_bool(value) -> bool:
    """Convertir a booleano un argumento de la línea de comandos."""
    return str(value).strip().lower() in ('true', '1', 'yes', 'si', 'sí', 'on')


def _setup(context, *args, **kwargs):
    """Construir las acciones una vez resueltos los argumentos."""
    del args, kwargs
    # Import diferido: sólo se necesita MoveIt cuando se lanza este archivo, no para
    # compilar ni probar el package.
    from moveit_configs_utils import MoveItConfigsBuilder

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    # Las mismas variables que usa el driver, para que el modelo de MoveIt sea el del robot.
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
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    acciones = [
        LogInfo(msg='move_group en modo SÓLO PLANIFICACIÓN: la ejecución pasa por '
                    'moveit_safe_executor (burger_kinova_reference).'),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[moveit_config.to_dict(), {
                'allow_trajectory_execution': False,
                'publish_monitored_planning_scene': True,
                'use_sim_time': _as_bool(arg('use_sim_time')),
            }],
        ),
    ]
    if _as_bool(arg('use_fake_hardware')):
        acciones.append(LogInfo(
            msg='Modo fake: el driver se lanza sin la Robotiq 2F-85, así que move_group '
                'avisará de que falta robotiq_85_left_knuckle_joint en /joint_states. Es '
                'esperado: el brazo se planifica igual y la pinza se modela abierta.'))

    if _as_bool(arg('launch_rviz')):
        # Con el robot real, RViz comparte la CPU de la anfitriona con el ciclo de control
        # del driver, que en WSL no obtiene prioridad de tiempo real
        # (TROUBLESHOOTING.md §3.7).
        acciones.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_moveit',
            output='log',
            prefix=None if _as_bool(arg('use_fake_hardware')) else 'nice -n 19',
            arguments=['-d', os.path.join(get_package_share_directory(MOVEIT_CONFIG),
                                          'config', 'moveit.rviz')],
            parameters=[moveit_config.robot_description,
                        moveit_config.robot_description_semantic,
                        moveit_config.robot_description_kinematics,
                        moveit_config.planning_pipelines,
                        moveit_config.joint_limits],
        ))

    if _as_bool(arg('cargar_escena')):
        acciones.append(TimerAction(period=3.0, actions=[Node(
            package=PACKAGE_NAME,
            executable='escena_moveit',
            name='escena_moveit',
            output='screen',
            parameters=[{'obstacle_links': [link.strip() for link in
                                            arg('obstacle_links').split(',') if link.strip()]}],
        )]))
    return acciones


def generate_launch_description() -> LaunchDescription:
    """Declarar los argumentos y delegar en :func:`_setup`."""
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='0.0.0.0',
                              description='IP del robot; sólo afecta al modelo cargado'),
        DeclareLaunchArgument('use_fake_hardware', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('cargar_escena', default_value='true',
                              description='Cargar la mesa de burger_description en MoveIt'),
        DeclareLaunchArgument('obstacle_links', default_value='table_link',
                              description='Eslabones de la escena que son obstáculos, '
                                          'separados por comas'),
        OpaqueFunction(function=_setup),
    ])
