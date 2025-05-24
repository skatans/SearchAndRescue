import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

import time

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
        self.cmd_vel_publisher = self.create_publisher(Twist, '/Mavic_2_PRO/cmd_vel', 10)
        self.gps_publisher = self.create_publisher(String, '/target_found', 10)
        self.bridge = CvBridge()
        self.target_found = False
        self.arrived_at_target = False
        self.get_logger().info('Mavic 2 Pro Camera Node has been started.')
        self.mid_x = 0
        self.mid_y = 0

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.mid_x == 0 and self.mid_y == 0:
                height, width, _ = cv_image.shape
                self.mid_x = width // 2
                self.mid_y = height // 2
            cv2.imshow("Mavic 2 Pro Camera", cv_image)
            cv2.waitKey(1)
            if not self.target_found:
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
        time.sleep(1.5) # Stabilization delay
        self.stop()  # Stop after moving

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
        time.sleep(1.5) # Stabilization delay
        self.stop()  # Stop after turning

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
        time.sleep(1.5) # Stabilization delay
        self.stop()  # Stop after turning

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
        time.sleep(1.5) # Stabilization delay
        self.stop()  # Stop 

    def fly_around(self):
        # This method will contain logic to control the drone's flight
        # For now, we will just log that the drone is flying around
        self.get_logger().info('Mavic 2 Pro is flying around.')

        #self.move_forward()  # Move forward as a default action
            
    def identify_target(self, image):
        # Use the local haarcascade_fullbody.xml from the package's share directory
        package_share_directory = ament_index_python.get_package_share_directory('my_package')
        haarcascade_path = os.path.join(package_share_directory, 'data', 'haarcascade_fullbody.xml')
        human_cascade = cv2.CascadeClassifier(haarcascade_path)

        if human_cascade.empty():
            self.get_logger().error(f'Failed to load Haar cascade classifier from {haarcascade_path}')
            self.target_found = False
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        humans = human_cascade.detectMultiScale(gray, 1.1, 4)
        if len(humans) == 0:
            return
        else:
            if not self.target_found:
                self.get_logger().info('Target found for the first time.')
                self.target_found = True
            # Draw bounding boxes around the detected human
            for (x, y, w, h) in humans:
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Display the image with bounding boxes
            cv2.imshow("Mavic 2 Pro Detections", image)
            cv2.waitKey(1)
            
            # Fly towards the center of the detected human
            self.fly_to_target(x + w // 2, y + h // 2)

    def fly_to_target(self, target_x, target_y):
        # Calculate the offset from the center of the image
        offset_x = target_x - self.mid_x
        offset_y = target_y - self.mid_y

        # Adjust the drone's movement based on the offset
        if abs(offset_x) > 50:
            if offset_x < 0:
                self.turn_left()
            else:
                self.turn_right()
        elif abs(offset_y) > 50:
            if offset_y < 0:
                self.change_altitude(0.1)
            else:
                self.change_altitude(-0.1)
        else:
            self.move_forward()


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