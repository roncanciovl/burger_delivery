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
MoveIt 2 planifica; ``burger_kinova_reference`` valida y ejecuta.

``move_group`` se lanza con ``allow_trajectory_execution: false``: no puede mover el robot,
ni desde código ni con el botón *Execute* de RViz. Este nodo es la única puerta hacia el
``joint_trajectory_controller`` y aplica a cada plan las reglas del laboratorio:

1. Lee la telemetría con :class:`LinkHealth` (fresca, completa, creíble).
2. Pide el plan a ``move_group`` **desde la pose medida** (no desde la que MoveIt recuerda).
3. Valida la trayectoria completa (:mod:`trajectory_validation`): arranque, límites,
   velocidad, saltos de 2π, recorrido, duración, ``enable_motion``.
4. Muestra el informe y, en modo interactivo, pide confirmación escribiendo "si".
5. Envía la trayectoria a ``FollowJointTrajectory`` y la vigila: si la telemetría se pierde
   durante el movimiento, cancela la meta.
6. Al terminar, compara la pose final medida con la planificada.

Dos fuentes de metas (``source``):

* ``goals``: la secuencia de ``config/metas_moveit.yaml`` (articular, pose en un marco de
  ``burger_description``, cartesiana o pinza).
* ``rviz``: los planes que se calculan con el botón *Plan* del panel MotionPlanning de RViz
  (tópico ``/display_planned_path``). Cada plan nuevo pasa por la misma validación.

Modo seco (``dry_run:=true``, por defecto): planifica y valida, nunca envía nada.

Códigos de salida: 0 éxito · 1 bloqueado por seguridad · 2 infraestructura · 3 rechazado o
abortado por el controlador · 4 MoveIt no encontró plan.

Uso::

    ros2 run burger_kinova_reference moveit_safe_executor --ros-args \
        --params-file <config>/kinova_connection.yaml -p dry_run:=true
"""

import json
import math
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

from action_msgs.msg import GoalStatus

from builtin_interfaces.msg import Duration

from burger_kinova_reference.escena_moveit import default_scene_path
from burger_kinova_reference.escena_urdf import SceneModel
from burger_kinova_reference.link_metrics import LinkHealth, validate_joint_state
from burger_kinova_reference.logging_support import (
    declare_logging_parameters,
    log_logging_banner,
    ThrottledLogger,
)
from burger_kinova_reference.metas_moveit import (
    Meta,
    parse_metas,
    parse_secuencia,
    pose_en_marco_de_planificacion,
)
from burger_kinova_reference.safety import validate_safety_config
from burger_kinova_reference.trajectory_validation import (
    format_trajectory_report,
    reorder_points,
    validate_trajectory,
)

from control_msgs.action import FollowJointTrajectory, GripperCommand

from geometry_msgs.msg import Pose

from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    DisplayTrajectory,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionFK

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import JointState

from shape_msgs.msg import SolidPrimitive

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import yaml

EXIT_OK = 0
EXIT_BLOCKED = 1
EXIT_INFRASTRUCTURE = 2
EXIT_REJECTED = 3
EXIT_NO_PLAN = 4


def _segundos(duration) -> float:
    return duration.sec + duration.nanosec * 1e-9


def _duration(seconds: float) -> Duration:
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int(round((seconds - whole) * 1e9)))


class MoveItSafeExecutor(Node):
    """Planificación con MoveIt 2 y ejecución supervisada por las reglas del laboratorio."""

    def __init__(self):
        """Declarar parámetros y construir clientes, suscripciones y estado."""
        super().__init__('moveit_safe_executor')
        p = self.declare_parameter
        # ---- comunes con los demás clientes del package -------------------------------
        p('joint_state_topic', '/joint_states')
        p('trajectory_action_name', '/joint_trajectory_controller/follow_joint_trajectory')
        p('use_fake_hardware', True)
        p('enable_motion', False)
        p('dry_run', True)
        p('require_operator_confirmation', True)
        p('expected_joints', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        p('joint_state_timeout_s', 1.0)
        p('min_joint_state_hz', 20.0)
        p('max_plausible_joint_rad', 100.0)
        p('min_rate_observation_s', 0.5)
        p('joint_min_rad', [-3.14, -2.24, -2.57, -3.14, -2.09, -3.14])
        p('joint_max_rad', [3.14, 2.24, 2.57, 3.14, 2.09, 3.14])
        p('action_server_timeout_s', 10.0)
        # ---- propios de MoveIt ----------------------------------------------------------
        p('source', 'goals')
        p('goals_file', '')
        p('sequence', [''])
        p('pause_s', 1.0)
        p('planning_group', 'manipulator')
        p('ee_link', 'end_effector_link')
        p('planning_frame', 'base_link')
        p('plan_service', '/plan_kinematic_path')
        p('cartesian_service', '/compute_cartesian_path')
        p('fk_service', '/compute_fk')
        p('display_topic', '/display_planned_path')
        p('pipeline_id', 'ompl')
        p('planner_id', '')
        p('planning_time_s', 5.0)
        p('planning_attempts', 5)
        p('velocity_scaling', 0.1)
        p('acceleration_scaling', 0.1)
        p('position_tolerance_m', 0.005)
        p('orientation_tolerance_rad', 0.02)
        p('joint_goal_tolerance_rad', 0.001)
        p('min_cartesian_fraction', 0.99)
        # ---- reglas sobre la trayectoria completa --------------------------------------
        p('start_tolerance_rad', 0.02)
        p('max_velocity_rad_s', 0.35)
        p('max_joint_travel_rad', 3.0)
        p('max_trajectory_duration_s', 60.0)
        p('continuous_joints', ['joint_1', 'joint_4', 'joint_6'])
        p('final_tolerance_rad', 0.02)
        p('settle_s', 0.3)
        # ---- escena de burger_description (metas en sus marcos) ---------------------------
        p('scene_urdf', default_scene_path())
        p('anchor_link', 'world')
        # ---- pinza ----------------------------------------------------------------------
        p('gripper_action_name', '/robotiq_gripper_controller/gripper_cmd')
        p('gripper_min', 0.0)
        p('gripper_max', 0.8)
        p('gripper_max_effort', 50.0)
        p('gripper_timeout_s', 10.0)
        # ---- evidencia ------------------------------------------------------------------
        p('report_file', '')
        declare_logging_parameters(self)

        self._log = ThrottledLogger(
            self.get_logger(), float(self.get_parameter('log_throttle_period_s').value))
        log_logging_banner(self)

        g = self.get_parameter
        self._joints: List[str] = list(g('expected_joints').value)
        self._joint_min = list(g('joint_min_rad').value)
        self._joint_max = list(g('joint_max_rad').value)
        self._dry_run = bool(g('dry_run').value)
        self._enable_motion = bool(g('enable_motion').value)
        self._fake = bool(g('use_fake_hardware').value)
        self._frame = str(g('planning_frame').value)
        self._group = str(g('planning_group').value)
        self._ee = str(g('ee_link').value)

        self._health = LinkHealth(
            expected_joints=self._joints,
            timeout_s=float(g('joint_state_timeout_s').value),
            min_hz=float(g('min_joint_state_hz').value),
            min_span_s=float(g('min_rate_observation_s').value),
            max_abs_rad=float(g('max_plausible_joint_rad').value),
        )
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._rviz_plans: List[JointTrajectory] = []
        self._rviz_event = threading.Event()
        self._active_goal = None
        # En modo seco el brazo no se mueve: para que la secuencia encadene, cada meta se
        # planifica desde la pose FINAL del plan anterior (pose virtual), no desde la real.
        self._virtual: Optional[Dict[str, float]] = None

        grupo = ReentrantCallbackGroup()
        self.create_subscription(JointState, str(g('joint_state_topic').value),
                                 self._on_joint_state, qos_profile_sensor_data,
                                 callback_group=grupo)
        self._plan_client = self.create_client(GetMotionPlan, str(g('plan_service').value),
                                               callback_group=grupo)
        self._cartesian_client = self.create_client(
            GetCartesianPath, str(g('cartesian_service').value), callback_group=grupo)
        self._fk_client = self.create_client(GetPositionFK, str(g('fk_service').value),
                                             callback_group=grupo)
        self._trajectory_client = ActionClient(
            self, FollowJointTrajectory, str(g('trajectory_action_name').value),
            callback_group=grupo)
        self._gripper_client = ActionClient(
            self, GripperCommand, str(g('gripper_action_name').value), callback_group=grupo)
        if str(g('source').value) == 'rviz':
            self.create_subscription(DisplayTrajectory, str(g('display_topic').value),
                                     self._on_display, 10, callback_group=grupo)

        modo = 'FAKE' if self._fake else 'HARDWARE REAL'
        self.get_logger().info(
            f'moveit_safe_executor | fuente={g("source").value} | modo={modo} | '
            f'dry_run={self._dry_run} | enable_motion={self._enable_motion} | '
            f'velocidad MoveIt={g("velocity_scaling").value:.2f} | '
            f'v. máx. validada={g("max_velocity_rad_s").value:.2f} rad/s')

    # ------------------------------------------------------------------ telemetría ------
    def _on_joint_state(self, msg: JointState) -> None:
        validacion = validate_joint_state(
            msg.name, msg.position, self._joints,
            float(self.get_parameter('max_plausible_joint_rad').value))
        with self._lock:
            self._health.update(validacion, time.monotonic())

    def _snapshot(self) -> Tuple[Dict[str, float], bool]:
        with self._lock:
            return dict(self._health.positions), self._health.is_fresh(time.monotonic())

    def _wait_telemetry(self, timeout_s: float = 10.0) -> bool:
        limite = time.monotonic() + timeout_s
        while time.monotonic() < limite and not self._stop.is_set():
            posiciones, fresca = self._snapshot()
            with self._lock:
                fiable = self._health.rate_is_reliable()
            if fresca and fiable and len(posiciones) == len(self._joints):
                return True
            time.sleep(0.1)
        return False

    def _on_display(self, msg: DisplayTrajectory) -> None:
        if msg.trajectory:
            with self._lock:
                self._rviz_plans = [msg.trajectory[0].joint_trajectory]
            self._rviz_event.set()

    # ------------------------------------------------------------------- ejecución ------
    def run(self) -> int:
        """Ejecutar según ``source``; devuelve el código de salida."""
        g = self.get_parameter
        errores = validate_safety_config(
            self._joints, [0.0] * len(self._joints), self._joint_min, self._joint_max,
            0.1, 1.0)
        if errores:
            for error in errores:
                self.get_logger().error(f'[CONFIGURACIÓN INSEGURA] {error}')
            return EXIT_BLOCKED

        self.get_logger().info('Esperando telemetría fresca de /joint_states...')
        if not self._wait_telemetry():
            self.get_logger().error('[INFRAESTRUCTURA] No llegó telemetría válida: revisa el '
                                    'driver, el ROS_DOMAIN_ID y la red DDS.')
            return EXIT_INFRASTRUCTURE

        if str(g('source').value) == 'rviz':
            return self._run_rviz()
        return self._run_goals()

    def _run_goals(self) -> int:
        g = self.get_parameter
        try:
            with open(str(g('goals_file').value), encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
            metas = parse_metas(data, len(self._joints))
            pedidas = [n for n in g('sequence').value if n]
            secuencia = parse_secuencia(data, metas, pedidas)
        except (OSError, ValueError, yaml.YAMLError) as error:
            self.get_logger().error(f'[METAS] {error}')
            return EXIT_BLOCKED
        modelo = None
        if any(m.tipo == 'pose' for m in secuencia):
            try:
                modelo = SceneModel.from_file(str(g('scene_urdf').value))
            except (OSError, ValueError) as error:
                self.get_logger().warn(f'[ESCENA] No se pudo leer la escena: {error}')

        self.get_logger().info('Secuencia: ' + ' → '.join(m.nombre for m in secuencia))
        for indice, meta in enumerate(secuencia, start=1):
            if self._stop.is_set():
                return EXIT_BLOCKED
            self.get_logger().info(f'══ Meta {indice}/{len(secuencia)}: {meta.nombre} '
                                   f'({meta.tipo}) ══')
            codigo = self._ejecutar_meta(meta, modelo)
            if codigo != EXIT_OK:
                self.get_logger().error(
                    f'[SECUENCIA] Detenida en "{meta.nombre}" (código {codigo}). No se '
                    'reintenta: este proyecto no recupera fallas automáticamente.')
                return codigo
            if indice < len(secuencia):
                time.sleep(float(g('pause_s').value))
        self.get_logger().info(f'[SECUENCIA] Completada: {len(secuencia)} meta(s).')
        return EXIT_OK

    def _run_rviz(self) -> int:
        self.get_logger().info(
            'Modo RViz: planifica con el botón "Plan" del panel MotionPlanning. Cada plan '
            'se valida aquí; "Execute" de RViz está deshabilitado en move_group. Ctrl+C sale.')
        ultimo = EXIT_OK
        while not self._stop.is_set():
            if not self._rviz_event.wait(timeout=0.5):
                continue
            self._rviz_event.clear()
            with self._lock:
                plan = self._rviz_plans[0]
            ultimo = self._validar_y_ejecutar('plan de RViz', plan)
            self.get_logger().info(f'Plan de RViz terminado con código {ultimo}. Esperando '
                                   'el siguiente...')
        return ultimo

    def _ejecutar_meta(self, meta: Meta, modelo: Optional[SceneModel]) -> int:
        if meta.tipo == 'pinza':
            return self._pinza(meta)
        actual, fresca = self._snapshot()
        if self._dry_run and self._virtual is not None:
            actual = dict(self._virtual)
        if not fresca:
            self.get_logger().error('[SEGURIDAD] Telemetría vencida antes de planificar.')
            return EXIT_BLOCKED
        try:
            if meta.tipo == 'articular':
                plan = self._plan(self._constraints_articulares(meta), actual)
            elif meta.tipo == 'pose':
                xyz, q = pose_en_marco_de_planificacion(
                    meta, modelo, str(self.get_parameter('anchor_link').value), self._frame)
                self.get_logger().info(
                    f'Pose objetivo en {self._frame}: ({xyz[0]:+.3f}, {xyz[1]:+.3f}, '
                    f'{xyz[2]:+.3f}) m')
                plan = self._plan(self._constraints_pose(xyz, q), actual)
            else:
                plan = self._plan_cartesiano(meta.desplazamiento, actual)
        except ValueError as error:
            self.get_logger().error(f'[METAS] {error}')
            return EXIT_BLOCKED
        except RuntimeError as error:
            self.get_logger().error(f'[MOVEIT] {error}')
            return EXIT_NO_PLAN
        return self._validar_y_ejecutar(meta.nombre, plan)

    # --------------------------------------------------------------- planificación ------
    def _start_state(self, actual: Dict[str, float]) -> RobotState:
        state = RobotState()
        state.joint_state.name = list(self._joints)
        state.joint_state.position = [float(actual[j]) for j in self._joints]
        state.is_diff = False
        return state

    def _constraints_articulares(self, meta: Meta) -> Constraints:
        tol = float(self.get_parameter('joint_goal_tolerance_rad').value)
        constraints = Constraints(name=meta.nombre)
        for joint, value in zip(self._joints, meta.posiciones):
            constraints.joint_constraints.append(JointConstraint(
                joint_name=joint, position=float(value), tolerance_above=tol,
                tolerance_below=tol, weight=1.0))
        return constraints

    def _constraints_pose(self, xyz, q) -> Constraints:
        g = self.get_parameter
        target = Pose()
        target.position.x, target.position.y, target.position.z = xyz
        (target.orientation.x, target.orientation.y,
         target.orientation.z, target.orientation.w) = q
        position = PositionConstraint(link_name=self._ee, weight=1.0)
        position.header.frame_id = self._frame
        position.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE,
                                       dimensions=[float(g('position_tolerance_m').value)])],
            primitive_poses=[target])
        tol = float(g('orientation_tolerance_rad').value)
        orientation = OrientationConstraint(
            link_name=self._ee, orientation=target.orientation, weight=1.0,
            absolute_x_axis_tolerance=tol, absolute_y_axis_tolerance=tol,
            absolute_z_axis_tolerance=tol)
        orientation.header.frame_id = self._frame
        return Constraints(position_constraints=[position],
                           orientation_constraints=[orientation])

    def _call(self, client, request, nombre: str, timeout_s: float = 15.0):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'move_group no ofrece {nombre}; ¿está lanzado '
                               'moveit_planificacion.launch.py?')
        future = client.call_async(request)
        if not self._await(future, timeout_s):
            raise RuntimeError(f'{nombre} no respondió en {timeout_s:.0f} s')
        return future.result()

    def _plan(self, goal: Constraints, actual: Dict[str, float]) -> JointTrajectory:
        g = self.get_parameter
        req = MotionPlanRequest()
        req.group_name = self._group
        req.start_state = self._start_state(actual)
        req.goal_constraints = [goal]
        req.pipeline_id = str(g('pipeline_id').value)
        req.planner_id = str(g('planner_id').value)
        req.num_planning_attempts = int(g('planning_attempts').value)
        req.allowed_planning_time = float(g('planning_time_s').value)
        req.max_velocity_scaling_factor = float(g('velocity_scaling').value)
        req.max_acceleration_scaling_factor = float(g('acceleration_scaling').value)
        response = self._call(self._plan_client, GetMotionPlan.Request(motion_plan_request=req),
                              'plan_kinematic_path',
                              timeout_s=float(g('planning_time_s').value) + 10.0)
        result = response.motion_plan_response
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'MoveIt no encontró plan (código {result.error_code.val}); '
                               'revisa alcance, colisiones con la escena y límites')
        self.get_logger().info(f'Plan encontrado en {result.planning_time:.2f} s')
        return result.trajectory.joint_trajectory

    def _plan_cartesiano(self, desplazamiento, actual: Dict[str, float]) -> JointTrajectory:
        g = self.get_parameter
        fk = GetPositionFK.Request(fk_link_names=[self._ee],
                                   robot_state=self._start_state(actual))
        fk.header.frame_id = self._frame
        fk_result = self._call(self._fk_client, fk, 'compute_fk')
        if fk_result.error_code.val != MoveItErrorCodes.SUCCESS or not fk_result.pose_stamped:
            raise RuntimeError('compute_fk no devolvió la pose del efector')
        destino = fk_result.pose_stamped[0].pose
        destino.position.x += desplazamiento[0]
        destino.position.y += desplazamiento[1]
        destino.position.z += desplazamiento[2]
        req = GetCartesianPath.Request(
            start_state=self._start_state(actual), group_name=self._group,
            link_name=self._ee, waypoints=[destino], max_step=0.005, avoid_collisions=True)
        req.header.frame_id = self._frame
        req.max_velocity_scaling_factor = float(g('velocity_scaling').value)
        req.max_acceleration_scaling_factor = float(g('acceleration_scaling').value)
        result = self._call(self._cartesian_client, req, 'compute_cartesian_path')
        fraccion = float(result.fraction)
        if fraccion < float(g('min_cartesian_fraction').value):
            raise RuntimeError(f'la línea recta sólo es alcanzable al {100 * fraccion:.0f} %: '
                               'se interrumpe por colisión, límite o singularidad')
        self.get_logger().info(f'Trayectoria cartesiana: {100 * fraccion:.0f} % del tramo')
        return result.solution.joint_trajectory

    # ----------------------------------------------------- validación y ejecución -------
    def _validar_y_ejecutar(self, nombre: str, plan: JointTrajectory) -> int:
        g = self.get_parameter
        try:
            puntos = reorder_points(
                plan.joint_names,
                [(_segundos(pt.time_from_start), list(pt.positions), list(pt.velocities))
                 for pt in plan.points],
                self._joints)
        except ValueError as error:
            self.get_logger().error(f'[SEGURIDAD] {error}')
            return EXIT_BLOCKED

        actual, fresca = self._snapshot()
        if self._dry_run and self._virtual is not None and str(g('source').value) == 'goals':
            actual = dict(self._virtual)
            self.get_logger().info('[MODO SECO] Se valida desde la pose final del plan '
                                   'anterior (el brazo no se movió).')
        validacion, resumen = validate_trajectory(
            expected_joints=self._joints, points=puntos, current_positions=actual,
            joint_min=self._joint_min, joint_max=self._joint_max,
            start_tolerance_rad=float(g('start_tolerance_rad').value),
            max_velocity_rad_s=float(g('max_velocity_rad_s').value),
            max_joint_travel_rad=float(g('max_joint_travel_rad').value),
            max_duration_s=float(g('max_trajectory_duration_s').value),
            # En modo seco se valida la geometría; enable_motion se informa aparte.
            enable_motion=self._enable_motion or self._dry_run, use_fake_hardware=self._fake,
            telemetry_fresh=fresca, continuous_joints=list(g('continuous_joints').value))
        for linea in format_trajectory_report(self._joints, actual, validacion, resumen):
            self.get_logger().info(linea)
        registro = {'meta': nombre, 'puntos': resumen.points,
                    'duracion_s': resumen.duration_s, 'recorrido_rad': resumen.travel,
                    'v_pico_rad_s': resumen.peak_velocity, 'permitida': validacion.allowed,
                    'bloqueos': validacion.blocks, 'dry_run': self._dry_run}

        if not resumen.moves and validacion.allowed:
            self._registrar(dict(registro, resultado='SIN_MOVIMIENTO'))
            return EXIT_OK
        if self._dry_run:
            self.get_logger().info(f'[MODO SECO] "{nombre}" planificada y validada; no se '
                                   'contacta el controlador.')
            if not self._enable_motion:
                self.get_logger().warn('[MODO SECO] Al ejecutar, enable_motion=false '
                                       'bloquearía el envío.')
            self._registrar(dict(registro, resultado='MODO_SECO'))
            if validacion.allowed:
                self._virtual = dict(resumen.final)
            return EXIT_OK if validacion.allowed else EXIT_BLOCKED
        if not validacion.allowed:
            self.get_logger().error(f'[SEGURIDAD] "{nombre}" bloqueada: {validacion.reason}')
            self._registrar(dict(registro, resultado='BLOQUEADA'))
            return EXIT_BLOCKED
        if not self._confirmar(nombre, resumen.duration_s):
            self.get_logger().warn('[SEGURIDAD] Envío cancelado por el operador.')
            self._registrar(dict(registro, resultado='CANCELADA_OPERADOR'))
            return EXIT_BLOCKED

        codigo, detalle = self._enviar(nombre, puntos, resumen.duration_s)
        time.sleep(float(g('settle_s').value))   # deja llegar el último /joint_states
        final, _ = self._snapshot()
        desvio = max((abs(final[j] - resumen.final[j]) for j in self._joints
                      if j in final), default=float('nan'))
        if codigo == EXIT_OK:
            nivel = (self.get_logger().info
                     if desvio <= float(g('final_tolerance_rad').value)
                     else self.get_logger().warn)
            nivel(f'[RESULTADO] "{nombre}" completada; desvío final máximo '
                  f'{desvio:.4f} rad respecto al plan')
        self._registrar(dict(registro, resultado=detalle, desvio_final_rad=desvio))
        return codigo

    def _confirmar(self, nombre: str, duracion: float) -> bool:
        if not bool(self.get_parameter('require_operator_confirmation').value):
            return True
        if not sys.stdin or not sys.stdin.isatty():
            self.get_logger().warn(
                '[SEGURIDAD] Ejecución no interactiva: se omite la confirmación por teclado. '
                'La autorización queda registrada en enable_motion:=true y dry_run:=false.')
            return True
        destino = 'HARDWARE REAL' if not self._fake else 'hardware simulado'
        try:
            respuesta = input(f'\n¿Ejecutar "{nombre}" ({duracion:.1f} s) en el {destino}? '
                              'Escribe "si" para continuar: ')
        except (EOFError, KeyboardInterrupt):
            return False
        return respuesta.strip().lower() in ('si', 'sí', 's', 'yes', 'y')

    def _enviar(self, nombre: str, puntos, duracion: float) -> Tuple[int, str]:
        timeout = float(self.get_parameter('action_server_timeout_s').value)
        if not self._trajectory_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('[INFRAESTRUCTURA] El controlador de trayectoria no '
                                    'responde: ¿corre el driver en la anfitriona?')
            return EXIT_INFRASTRUCTURE, 'SIN_CONTROLADOR'
        trayectoria = JointTrajectory(joint_names=list(self._joints))
        for t, q, v in puntos:
            punto = JointTrajectoryPoint(positions=[float(x) for x in q],
                                         time_from_start=_duration(t))
            if v:
                punto.velocities = [float(x) for x in v]
            trayectoria.points.append(punto)
        goal = FollowJointTrajectory.Goal(trajectory=trayectoria)

        self.get_logger().warn(f'[ENVÍO] "{nombre}": {len(puntos)} puntos, {duracion:.1f} s')
        futuro = self._trajectory_client.send_goal_async(goal, feedback_callback=self._feedback)
        if not self._await(futuro, timeout) or futuro.result() is None:
            return EXIT_INFRASTRUCTURE, 'SIN_RESPUESTA'
        handle = futuro.result()
        if not handle.accepted:
            self.get_logger().error(f'[RESULTADO] "{nombre}" RECHAZADA por el controlador')
            return EXIT_REJECTED, 'RECHAZADA'
        with self._lock:
            self._active_goal = handle
        try:
            return self._seguir(nombre, handle, duracion)
        except KeyboardInterrupt:
            # Se cancela aquí, con la meta todavía a mano: el finally la olvida antes de
            # que main() llegue a solicitar_parada().
            self.solicitar_parada()
            raise
        finally:
            with self._lock:
                self._active_goal = None

    def _seguir(self, nombre: str, handle, duracion: float) -> Tuple[int, str]:
        """Vigilar la meta en curso hasta su resultado; cancelarla si algo falla."""
        resultado = handle.get_result_async()
        limite = time.monotonic() + duracion + 15.0
        while not resultado.done():
            _, fresca = self._snapshot()
            if not fresca or self._stop.is_set() or time.monotonic() > limite:
                motivo = ('telemetría perdida durante el movimiento' if not fresca else
                          'interrupción del operador' if self._stop.is_set() else
                          'el controlador no terminó a tiempo')
                self.get_logger().error(f'[SEGURIDAD] {motivo}: se CANCELA la meta.')
                self._await(handle.cancel_goal_async(), 5.0)
                return EXIT_INFRASTRUCTURE if not fresca else EXIT_REJECTED, 'CANCELADA'
            time.sleep(0.05)
        envuelto = resultado.result()
        error = envuelto.result.error_code
        if (envuelto.status == GoalStatus.STATUS_SUCCEEDED
                and error == FollowJointTrajectory.Result.SUCCESSFUL):
            return EXIT_OK, 'EJECUTADA'
        self.get_logger().error(
            f'[RESULTADO] "{nombre}" terminó en estado {envuelto.status}, error_code={error}, '
            f'"{envuelto.result.error_string}"')
        return EXIT_REJECTED, f'ABORTADA({error})'

    def _feedback(self, msg) -> None:
        errores = list(msg.feedback.error.positions or [])
        pico = max((abs(v) for v in errores), default=0.0)
        self._log.info(f'[FEEDBACK] error de seguimiento máximo={pico:.4f} rad', throttle=True)

    def _pinza(self, meta: Meta) -> int:
        g = self.get_parameter
        lo, hi = float(g('gripper_min').value), float(g('gripper_max').value)
        if not (lo <= meta.pinza <= hi) or not math.isfinite(meta.pinza):
            self.get_logger().error(f'[SEGURIDAD] Pinza: {meta.pinza} fuera de [{lo}, {hi}]')
            return EXIT_BLOCKED
        if self._dry_run:
            self.get_logger().info(f'[MODO SECO] Pinza a {meta.pinza:.3f}: no se envía.')
            return EXIT_OK
        if not self._enable_motion:
            self.get_logger().error('[SEGURIDAD] Pinza: movimiento deshabilitado '
                                    '(enable_motion=false)')
            return EXIT_BLOCKED
        if not self._gripper_client.wait_for_server(
                timeout_sec=float(g('action_server_timeout_s').value)):
            self.get_logger().error(
                '[INFRAESTRUCTURA] No hay controlador de pinza. En modo fake el launch de '
                'conexión omite la Robotiq 2F-85 (ver kinova_connection.launch.py).')
            return EXIT_INFRASTRUCTURE
        if not self._confirmar(meta.nombre, 0.0):
            return EXIT_BLOCKED
        goal = GripperCommand.Goal()
        goal.command.position = float(meta.pinza)
        goal.command.max_effort = float(g('gripper_max_effort').value)
        futuro = self._gripper_client.send_goal_async(goal)
        if not self._await(futuro, 5.0) or not futuro.result().accepted:
            return EXIT_REJECTED
        resultado = futuro.result().get_result_async()
        if not self._await(resultado, float(g('gripper_timeout_s').value)):
            return EXIT_REJECTED
        r = resultado.result().result
        self.get_logger().info(f'[PINZA] posición={r.position:.3f} esfuerzo={r.effort:.1f} '
                               f'bloqueada={r.stalled} alcanzó={r.reached_goal}')
        return EXIT_OK

    def _registrar(self, registro: dict) -> None:
        ruta = str(self.get_parameter('report_file').value)
        if not ruta:
            return
        os.makedirs(os.path.dirname(os.path.abspath(ruta)), exist_ok=True)
        registro['fecha'] = time.strftime('%Y-%m-%dT%H:%M:%S')
        with open(ruta, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(registro, ensure_ascii=False) + '\n')

    def solicitar_parada(self) -> None:
        """Pedir que no se envíe nada más y cancelar la meta en curso, si la hay."""
        self._stop.set()
        with self._lock:
            handle = self._active_goal
        if handle is not None:
            self.get_logger().warn('[SEGURIDAD] Cancelando la trayectoria en curso: el '
                                   'controlador detiene el brazo.')
            self._await(handle.cancel_goal_async(), 3.0)

    @staticmethod
    def _await(futuro, timeout_s: float) -> bool:
        limite = time.monotonic() + timeout_s
        while time.monotonic() < limite:
            if futuro.done():
                return True
            time.sleep(0.02)
        return futuro.done()


def main(args: Optional[List[str]] = None) -> None:
    """Punto de entrada del ejecutable ``moveit_safe_executor``."""
    # Sin el manejador de señales de rclpy: con él, Ctrl+C apaga el contexto antes de que
    # se pueda cancelar la trayectoria en curso. Así Ctrl+C llega como KeyboardInterrupt,
    # se cancela la meta con el ejecutor todavía girando y sólo después se cierra.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    nodo: Optional[MoveItSafeExecutor] = None
    ejecutor = MultiThreadedExecutor(num_threads=4)
    codigo = EXIT_INFRASTRUCTURE
    hilo = None
    try:
        nodo = MoveItSafeExecutor()
        ejecutor.add_node(nodo)
        hilo = threading.Thread(target=ejecutor.spin, daemon=True)
        hilo.start()
        codigo = nodo.run()
    except KeyboardInterrupt:
        if nodo is not None:
            nodo.get_logger().warn('Interrupción por teclado: no se envía nada más.')
            nodo.solicitar_parada()
        codigo = EXIT_BLOCKED
    finally:
        try:
            ejecutor.shutdown()
            if hilo is not None:
                hilo.join(timeout=2.0)
            if nodo is not None:
                nodo.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except BaseException:  # noqa: B036 - cierre best-effort
            pass
    sys.exit(codigo)


if __name__ == '__main__':
    main()
