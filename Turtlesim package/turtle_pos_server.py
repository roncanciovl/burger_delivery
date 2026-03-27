import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
import math
import time

class TurtlePosServer(Node):
    def __init__(self):
        super().__init__('turtle_pos_server')
        
        # Separación por grupos para habilitar ejecución multihilo en ROS 2.
        # Esto permite que el servicio espere internamente mientras el control sigue calculando en paralelo.
        self.cb_group_timer = MutuallyExclusiveCallbackGroup()
        self.cb_group_service = MutuallyExclusiveCallbackGroup()
        
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10, callback_group=self.cb_group_timer)
        
        # Reutilizamos "TeleportAbsolute" de turtlesim para no tener que crear un paquete custom de interfaz (.srv)
        self.srv = self.create_service(
            TeleportAbsolute, 
            '/pos_simulator/go_to_pose', 
            self.handle_go_to_pose,
            callback_group=self.cb_group_service
        )
        
        self.pose = Pose()
        self.target_x = None
        self.target_y = None
        self.target_theta = None
        
        # Bucle de publicación (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop, callback_group=self.cb_group_timer)
        self.is_moving = False
        self.pose_received = False
        
        self.get_logger().info("✅ Servidor POS inicializado y esperando peticiones en '/pos_simulator/go_to_pose'")

    def update_pose(self, data):
        self.pose = data
        self.pose_received = True

    def handle_go_to_pose(self, request, response):
        """Este método se dispara cuando recibe un Request desde un Cliente ROS 2"""
        self.get_logger().info(f"Petición de Cliente recibida: Mover tortuga a X={request.x:.2f}, Y={request.y:.2f}, Theta={(math.degrees(request.theta)):.1f}°")
        
        if not self.pose_received:
            self.get_logger().warn("No hay datos de la tortuga. Abortando petición.")
            return response
            
        self.target_x = float(request.x)
        self.target_y = float(request.y)
        self.target_theta = float(request.theta)
        
        # Activar el movimiento gestionado por el control_loop
        self.is_moving = True
        
        self.get_logger().info("Viajando al destino... (Bloqueando respuesta hasta finalizar)")
        
        # Bloquea EXCLUSIVAMENTE este hilo del servicio, permitiendo que MultiThreadedExecutor mueva los motores 
        while self.is_moving and rclpy.ok():
            time.sleep(0.1)
            
        self.get_logger().info("Viaje finalizado con éxito. Devolviendo Response de Acción Completa al cliente.")
        return response

    def control_loop(self):
        """Bucle matemático P-Control corriendo constantemente de fondo"""
        if not self.is_moving or self.target_x is None:
            return

        distance = math.sqrt(pow((self.target_x - self.pose.x), 2) + pow((self.target_y - self.pose.y), 2))
        msg = Twist()

        if distance > 0.1:
            angle_to_target = math.atan2(self.target_y - self.pose.y, self.target_x - self.pose.x)
            angle_diff = angle_to_target - self.pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            if abs(angle_diff) > 0.5:
                msg.linear.x = 0.0
                msg.angular.z = 2.0 * angle_diff
            else:
                msg.linear.x = 1.5 * distance
                msg.angular.z = 4.0 * angle_diff
        else:
            angle_diff = self.target_theta - self.pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            if abs(angle_diff) > 0.05: 
                msg.linear.x = 0.0
                msg.angular.z = 2.0 * angle_diff
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.is_moving = False

        if msg.linear.x > 3.0: msg.linear.x = 3.0
        if msg.angular.z > 3.0: msg.angular.z = 3.0
        elif msg.angular.z < -3.0: msg.angular.z = -3.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    server = TurtlePosServer()
    
    # Fundamental: Utilizar MultiThreadedExecutor con ROS 2 para servicios bloqueantes
    # en Python. Permite N hilos para procesar callbacks paralelamente (Timer y Srv).
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
