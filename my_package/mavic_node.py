import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MavicNode(Node):
    def __init__(self):
        super().__init__('mavic_camera_node')
        self.subscription = self.create_subscription(
            Image,
            '/Mavic_2_PRO/camera/image_color',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Mavic 2 Pro Camera", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MavicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()