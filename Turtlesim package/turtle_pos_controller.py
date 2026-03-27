import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import threading
import time
import sys

class TurtleGoToPoseNode(Node):
    def __init__(self):
        super().__init__('turtle_go_to_pose')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        
        self.pose = Pose()
        self.target_x = None
        self.target_y = None
        self.target_theta = None
        
        # Bucle de control a 10 Hz (ejecuta control_loop cada 0.1s)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.is_moving = False
        self.pose_received = False
        
        self.get_logger().info("Nodo controlador inicializado. Buscando a la tortuga...")

    def update_pose(self, data):
        if not self.pose_received:
            self.get_logger().info("✅ ¡Enlace establecido con Turtlesim! (Pose recibida)")
        self.pose = data
        self.pose_received = True

    def control_loop(self):
        if not self.is_moving or self.target_x is None:
            return

        if not self.pose_received:
            self.get_logger().warn("⚠️ Esperando datos de la tortuga... ¿Está corriendo turtlesim_node?")
            return

        # Distancia euclidiana hacia el punto objetivo
        distance = math.sqrt(pow((self.target_x - self.pose.x), 2) + pow((self.target_y - self.pose.y), 2))
        
        msg = Twist()

        # Fase 1 y 2: Girar hacia el objetivo y avanzar
        if distance > 0.1:
            angle_to_target = math.atan2(self.target_y - self.pose.y, self.target_x - self.pose.x)
            angle_diff = angle_to_target - self.pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            if abs(angle_diff) > 0.5:
                # Girar en su propio eje si el ángulo es muy grande
                msg.linear.x = 0.0
                msg.angular.z = 2.0 * angle_diff
            else:
                # Avanzar y alinear simultáneamente
                msg.linear.x = 1.5 * distance
                msg.angular.z = 4.0 * angle_diff
                
        # Fase 3: Llegamos al punto, alinear la orientación final (Theta)
        else:
            angle_diff = self.target_theta - self.pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            # Tolerancia de orientación: ~3 grados (0.05 radianes)
            if abs(angle_diff) > 0.05: 
                msg.linear.x = 0.0
                msg.angular.z = 2.0 * angle_diff
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.is_moving = False
                self.get_logger().info(f"🎯 ¡Objetivo alcanzado! (x={self.pose.x:.2f}, y={self.pose.y:.2f}, theta={math.degrees(self.pose.theta):.0f}°)")

        # Límites de seguridad para la tortuga
        if msg.linear.x > 3.0:
            msg.linear.x = 3.0
        if msg.angular.z > 3.0:
            msg.angular.z = 3.0
        elif msg.angular.z < -3.0:
            msg.angular.z = -3.0

        self.publisher_.publish(msg)

    def go_to_pose(self, x, y, theta_deg):
        self.target_x = x
        self.target_y = y
        self.target_theta = math.radians(theta_deg)
        self.is_moving = True
        self.get_logger().info(f"🚀 Viajando hacia: X={x}, Y={y}, Theta={theta_deg}°")


def input_thread(node):
    time.sleep(1.5) # Pequeña pausa para que los logs de inicio de ROS no pisen el texto
    print("\n" + "="*50)
    print("--- SIMULADOR DE POSICIONAMIENTO (POS) ---")
    print("="*50 + "\n")
    
    while rclpy.ok():
        try:
            x_str = input("\n> Ingresa X deseada (0-11) o 'q' para salir: ")
            if x_str.lower() == 'q':
                print("Saliendo...")
                rclpy.shutdown()
                sys.exit(0)
            
            x = float(x_str.replace(',', '.'))
            
            y_str = input("> Ingresa Y deseada (0-11): ")
            y = float(y_str.replace(',', '.'))
            
            theta_str = input("> Ingresa Theta en GRADOS (-180 a 180): ")
            theta = float(theta_str.replace(',', '.'))
            
            node.go_to_pose(x, y, theta)
            
            # Esperar hasta que termine
            while node.is_moving and rclpy.ok():
                time.sleep(0.1)
                
        except ValueError:
            print("\n[Error] Entrada inválida. Usa solo números.")
        except KeyboardInterrupt:
            rclpy.shutdown()
            sys.exit(0)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleGoToPoseNode()

    # Lanzamos el input en el hilo secundario
    thread = threading.Thread(target=input_thread, args=(turtle_controller,), daemon=True)
    thread.start()

    # ¡El bucle de ROS DEBE ir en el hilo principal para evitar fallos de tiempo!
    try:
        rclpy.spin(turtle_controller)
    except KeyboardInterrupt:
        pass
    finally:
        turtle_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
