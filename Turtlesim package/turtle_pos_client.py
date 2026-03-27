import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
import math
import sys

class TurtlePosClient(Node):
    def __init__(self):
        super().__init__('turtle_pos_client')
        # Crear cliente hacia el servicio '/pos_simulator/go_to_pose' de nuestro Servidor POS
        # Usamos el tipo estandar 'TeleportAbsolute' de turtlesim porque tiene los campos 'x', 'y', 'theta' requeridos
        self.cli = self.create_client(TeleportAbsolute, '/pos_simulator/go_to_pose')

    def send_request(self, x, y, theta_rad):
        # Esperar hasta que el ROS Service de nuestro Servidor esté encendido
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Buscando servidor del Simulador POS...')
        
        # Instanciar el tipo de mensaje Request (Petición)
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta_rad)
        
        self.get_logger().info(f'Enviando Request al Servidor: X={x}, Y={y}, Theta={(math.degrees(theta_rad)):.1f}°')
        
        # Disparamos el request asincrónico a la red
        future = self.cli.call_async(req)
        
        # Frenamos el hilo de la terminal principal hasta que ROS nos despierte con la Respuesta (Response)
        # Esto nos brinda un aviso preciso de que el servidor (la tortuga) ya llegó al destino final
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client_node = TurtlePosClient()

    print("\n" + "="*50)
    print("--- CLIENTE SIMULADOR POS ---")
    print("El cliente delegará la responsabilidad del movimiento a 'turtle_pos_server.py'")
    print("="*50 + "\n")
    
    try:
        while rclpy.ok():
            print("\n------------------------------------------------")
            x_str = input("> Ingresa X deseada (0-11) o 'q' para salir: ")
            if x_str.lower() == 'q':
                break
            
            x = float(x_str.replace(',', '.'))
            
            y_str = input("> Ingresa Y deseada (0-11): ")
            y = float(y_str.replace(',', '.'))
            
            theta_str = input("> Ingresa Theta en GRADOS (-180 a 180): ")
            theta_deg = float(theta_str.replace(',', '.'))
            theta_rad = math.radians(theta_deg)
            
            # Enviar el Request al Servidor (Paso Bloqueante porque esperamos la confirmación)
            print("\n[⏳] Solicitud enviada. Esperando cumplimiento del Servidor...")
            
            client_node.send_request(x, y, theta_rad)
            
            print("✅ ¡Response recibido! El Servidor ha confirmado la llegada de la tortuga")

    except ValueError:
        print("\n[Error] Entrada inválida. Usa solo números numéricos.")
    except KeyboardInterrupt:
        pass
    finally:
        print("\nCerrando cliente...")
        client_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
