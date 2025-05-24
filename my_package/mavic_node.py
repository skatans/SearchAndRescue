import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

import ament_index_python
import os

class MavicNode(Node):
    def __init__(self):
        super().__init__('mavic_camera_node')
        self.subscription = self.create_subscription(
            Image,
            '/Mavic_2_PRO/camera/image_color',
            self.listener_callback,
            10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.target_found = False
        self.get_logger().info('Mavic 2 Pro Camera Node has been started.')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Mavic 2 Pro Camera", cv_image)
            cv2.waitKey(1)
            self.identify_target(cv_image)
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Published stop command to /cmd_vel.')

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 1.0  # Move forward
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Published forward velocity command to /cmd_vel.')

    def turn_left(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 1.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Published left turn command to /cmd_vel.')

    def turn_right(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -1.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Published right turn command to /cmd_vel.')

    def change_altitude(self, altitude):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = altitude
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'Published altitude change command to /cmd_vel with altitude {altitude}.')

    def fly_around(self):
        # This method will contain logic to control the drone's flight
        # For now, we will just log that the drone is flying around
        self.get_logger().info('Mavic 2 Pro is flying around.')

        #self.move_forward()  # Move forward as a default action
            
    def identify_target(self, image):
        # Use the local haarcascade_fullbody.xml from the installed package data directory
        package_share_directory = ament_index_python.get_package_share_directory('my_package')
        print(package_share_directory)
        haarcascade_path = os.path.join(package_share_directory, 'data', 'haarcascade_fullbody.xml')
        human_cascade = cv2.CascadeClassifier(haarcascade_path)

        if human_cascade.empty():
            self.get_logger().error(f'Failed to load Haar cascade classifier from {haarcascade_path}')
            self.target_found = False
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        humans = human_cascade.detectMultiScale(gray, 1.1, 4)
        self.target_found = len(humans) > 0

        if self.target_found:
            self.get_logger().info('Target found in the image.')
        else:
            self.get_logger().info('No target found in the image.')

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