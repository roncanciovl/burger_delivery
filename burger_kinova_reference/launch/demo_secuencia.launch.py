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
Demostración completa: secuencia autónoma, visualización en RViz y log en rqt_console.

Levanta de una sola vez las tres cosas que hacen falta para ver el sistema funcionando:

* el **driver** del Kinova, con RViz mostrando la pose real del brazo;
* ``rqt_console``, que filtra ``/rosout`` por nodo y por severidad — ahí se ve la
  validación de cada punto, los envíos, el feedback y las transiciones del enlace;
* el **monitor** y el **cliente de secuencia**, arrancado con retraso para dar tiempo a
  que los controladores queden activos.

Modo seco, que **no mueve el robot** y sirve para ensayar la demostración::

    ros2 launch burger_kinova_reference demo_secuencia.launch.py \
        robot_ip:=192.168.1.10 use_fake_hardware:=false

Ejecución real, con espacio despejado y parada de emergencia accesible::

    ros2 launch burger_kinova_reference demo_secuencia.launch.py \
        robot_ip:=192.168.1.10 use_fake_hardware:=false \
        dry_run:=false enable_motion:=true

La secuencia por defecto vive en ``config/kinova_connection.yaml``
(``sequence_deltas_rad``) y está pensada para ser visible en RViz sin salirse de
``max_joint_delta_rad`` en ningún tramo.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

PACKAGE_NAME = 'burger_kinova_reference'


def _as_bool(value) -> bool:
    """Convertir a booleano un valor de la línea de comandos."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('true', '1', 'yes', 'si', 'sí', 'on')


def _setup(context, *args, **kwargs):
    """Construir las acciones una vez resueltas las sustituciones."""
    del args, kwargs

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    config_file = arg('config_file')
    dry_run = _as_bool(arg('dry_run'))
    enable_motion = _as_bool(arg('enable_motion'))
    retraso = float(arg('sequence_delay_s'))

    acciones = []

    if not dry_run and enable_motion:
        acciones.append(LogInfo(msg=(
            '⚠ SECUENCIA REAL: el brazo se moverá de forma autónoma. Espacio despejado, '
            'parada de emergencia accesible y nadie en el área de trabajo. Ctrl+C detiene '
            'la secuencia antes del siguiente punto; la meta en curso la termina el '
            'controlador.'
        )))
    else:
        acciones.append(LogInfo(msg=(
            'Modo de ensayo: la secuencia se valida punto por punto y NO se envía nada al '
            'servidor de acción. Para ejecutarla: dry_run:=false enable_motion:=true'
        )))

    # Driver + RViz + monitor, reutilizando el launch unificado.
    acciones.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(PACKAGE_NAME),
            'launch', 'kinova_connection.launch.py')),
        launch_arguments={
            'config_file': config_file,
            'start_driver': arg('start_driver'),
            'robot_ip': arg('robot_ip'),
            'use_fake_hardware': arg('use_fake_hardware'),
            'enable_motion': str(enable_motion).lower(),
            'launch_rviz': arg('launch_rviz'),
            'gripper': arg('gripper'),
            'dof': arg('dof'),
        }.items(),
    ))

    # rqt_console: la ventana de log. Filtra /rosout por nodo y severidad.
    if _as_bool(arg('launch_rqt_console')):
        acciones.append(Node(
            package='rqt_console', executable='rqt_console', name='rqt_console',
            output='screen',
        ))

    # La secuencia arranca con retraso: los spawners tardan en dejar activos el
    # broadcaster y el controlador de trayectoria, y el cliente exige telemetría creíble
    # antes de construir el primer punto.
    acciones.append(TimerAction(period=retraso, actions=[Node(
        package=PACKAGE_NAME, executable='safe_sequence_client',
        name='safe_sequence_client', output='screen', emulate_tty=True,
        parameters=[config_file, {
            'dry_run': dry_run,
            'enable_motion': enable_motion,
            'use_fake_hardware': _as_bool(arg('use_fake_hardware')),
        }],
    )]))

    return acciones


def generate_launch_description() -> LaunchDescription:
    """Declarar los argumentos y delegar el armado en :func:`_setup`."""
    default_config = os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'config', 'kinova_connection.yaml')

    declarados = [
        DeclareLaunchArgument('config_file', default_value=default_config,
                              description='YAML de configuración del enlace.'),
        DeclareLaunchArgument('start_driver', default_value='true',
                              choices=['true', 'false'],
                              description='Iniciar el driver en esta estación.'),
        DeclareLaunchArgument('robot_ip', default_value='0.0.0.0',
                              description='IP del Kinova.'),
        DeclareLaunchArgument('use_fake_hardware', default_value='true',
                              choices=['true', 'false'],
                              description='Validar sin movimiento físico.'),
        DeclareLaunchArgument('dof', default_value='6',
                              description='Grados de libertad del manipulador.'),
        DeclareLaunchArgument('gripper', default_value='robotiq_2f_85',
                              description='Pinza transferida al bringup.'),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                              choices=['true', 'false'],
                              description='Abrir RViz con la pose real del brazo.'),
        DeclareLaunchArgument('launch_rqt_console', default_value='true',
                              choices=['true', 'false'],
                              description='Abrir rqt_console con el log de /rosout.'),
        DeclareLaunchArgument('dry_run', default_value='true',
                              choices=['true', 'false'],
                              description='Validar la secuencia sin enviar ninguna meta.'),
        DeclareLaunchArgument('enable_motion', default_value='false',
                              choices=['true', 'false'],
                              description='Habilitación explícita de comandos físicos.'),
        DeclareLaunchArgument('sequence_delay_s', default_value='20.0',
                              description='Retraso antes de arrancar la secuencia, para '
                                          'dar tiempo a que los controladores se activen.'),
    ]
    return LaunchDescription(declarados + [OpaqueFunction(function=_setup)])
