import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("webcam_sub")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, "/video_stream", self.image_callback , qos_profile_sensor_data)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('Webcam Stream', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error('cv_bridge exception: %s' % e)


def main(args=None):
    rclpy.init(args=args)

    ip = ImageSubscriber()
    print("Subscribing...")
    rclpy.spin(ip)

    ip.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
