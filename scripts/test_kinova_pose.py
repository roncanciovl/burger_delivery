#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, JointConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand
from scipy.spatial.transform import Rotation as R
import sys
import argparse
import math

class KinovaTestNode(Node):
    def __init__(self):
        super().__init__('kinova_test_pose_cli')
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self._gripper_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')
        
        self.get_logger().info('Esperando al Action Server de MoveGroup...')
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server NO disponible!')
        else:
            self.get_logger().info('MoveGroup conectado.')
            
        self.get_logger().info('Esperando al Action Server de la Pinza (Gripper)...')
        if not self._gripper_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('GripperCommand action server NO disponible (Ignorar si usas modelo sin pinza).')
        else:
            self.get_logger().info('Gripper conectado.')

    def send_gripper_goal(self, position):
        goal_msg = GripperCommand.Goal()
        # position 0.0 es totalmente abierto, ~0.8 es totalmente cerrado (Robotiq 2F-85)
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = 50.0

        self.get_logger().info(f"Enviando Comando a la Pinza: position={position}")
        return self._gripper_client.send_goal_async(goal_msg)

    def send_cartesian_goal(self, x, y, z, roll=180.0, pitch=0.0, yaw=90.0, speed=0.05):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'manipulator'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        # ¡Aquí va la restricción de velocidad recomendada!
        goal_msg.request.max_velocity_scaling_factor = speed
        goal_msg.request.max_acceleration_scaling_factor = speed

        # --- Position Constraints ---
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'end_effector_link'
        target_point = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.001]
        target_point.primitives.append(primitive)
        target_pose = PoseStamped()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_point.primitive_poses.append(target_pose.pose)
        pos_constraint.constraint_region = target_point
        pos_constraint.weight = 1.0

        # --- Orientation Constraints ---
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'base_link'
        ori_constraint.link_name = 'end_effector_link'
        rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        quat = rot.as_quat()
        ori_constraint.orientation.x = quat[0]
        ori_constraint.orientation.y = quat[1]
        ori_constraint.orientation.z = quat[2]
        ori_constraint.orientation.w = quat[3]
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0

        goal_msg.request.goal_constraints.append(Constraints(
            name="goal_cartesiano",
            position_constraints=[pos_constraint],
            orientation_constraints=[ori_constraint]
        ))

        self.get_logger().info(f'Enviando Objetivo Cartesiano a MoveIt: X={x}, Y={y}, Z={z}')
        return self._move_group_client.send_goal_async(goal_msg)

    def send_joint_goal(self, joint_values, speed=0.05):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'manipulator'
        goal_msg.request.max_velocity_scaling_factor = speed
        goal_msg.request.max_acceleration_scaling_factor = speed
        
        constraints = Constraints()
        for i, val in enumerate(joint_values):
            jc = JointConstraint()
            jc.joint_name = f'joint_{i+1}'
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        self.get_logger().info(f'Enviando Estado de Joints (Home/Retract)...')
        return self._move_group_client.send_goal_async(goal_msg)

def main():
    parser = argparse.ArgumentParser(description='Herramienta de Diagnóstico: Mover Kinova Gen3 via CLI')
    parser.add_argument('--x', type=float, help='Coordenada X objetivo (en metros)')
    parser.add_argument('--y', type=float, help='Coordenada Y objetivo (en metros)')
    parser.add_argument('--z', type=float, help='Coordenada Z objetivo (en metros)')
    parser.add_argument('--roll', type=float, default=180.0, help='Ángulo Roll (grados, defecto: 180.0)')
    parser.add_argument('--pitch', type=float, default=0.0, help='Ángulo Pitch (grados, defecto: 0.0)')
    parser.add_argument('--yaw', type=float, default=90.0, help='Ángulo Yaw (grados, defecto: 90.0)')
    parser.add_argument('--state', choices=['Home', 'Retract', 'Vertical'], help='Mover a una pose de joint por defecto')
    parser.add_argument('--speed', type=float, default=0.05, help='Velocidad base (0.0 a 1.0, defecto: 0.05 suave)')
    parser.add_argument('--gripper', type=float, help='Apertura de la Pinza (0.0=abierto, 0.8=cerrado)')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = KinovaTestNode()

    future = None
    if args.gripper is not None:
        future = node.send_gripper_goal(args.gripper)
    elif args.state:
        states = {
            'Home': [0.0, 0.261, -2.27, 0.0, 0.96, 1.5708],
            'Retract': [0.0, -0.35, -2.54, 0.0, -0.87, 1.5708],
            'Vertical': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        }
        future = node.send_joint_goal(states[args.state], speed=args.speed)
    elif args.x is not None and args.y is not None and args.z is not None:
        future = node.send_cartesian_goal(args.x, args.y, args.z, args.roll, args.pitch, args.yaw, speed=args.speed)
    else:
        node.get_logger().error('Por favor, especifica Coordenadas X,Y,Z o escoge un estado --state / --gripper.')
        parser.print_help()
        rclpy.shutdown()
        return

    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().error('¡El Planificador ha rechazado el Goal! (Fuera del rango / Hay colisión)')
    else:
        node.get_logger().info('Goal aceptado. Kinova en movimiento...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        node.get_logger().info('¡Lanzamiento / Movimiento Completado!')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
