import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_pub')
        self.publisher_ = self.create_publisher(Image, '/video_stream', qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 FPS
        self.bridge = CvBridge()
        self.count = 0

    def timer_callback(self):
        # Create a black image
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Draw some moving text
        text = f"Streaming Test - Frame {self.count}"
        self.count += 1
        cv2.putText(frame, text, (50 + (self.count % 100), 240), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Add a timestamp
        curr_time = time.strftime("%H:%M:%S")
        cv2.putText(frame, curr_time, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

        # Convert and publish
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyPublisher()
    node.get_logger().info("Publishing dummy images for network test...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
