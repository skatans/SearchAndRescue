import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped

import cv2
import time
import math
import random
import numpy as np

import ament_index_python
import os

class MavicNode(Node):
    def __init__(self):
        super().__init__('mavic_camera_node')
        self.get_logger().info('Mavic 2 Pro Camera Node has been started.')
        # Subscription to the drone camera image topic
        self.img_sub = self.create_subscription(Image, '/Mavic_2_PRO/camera/image_color', self.listener_callback, 10)
        # Image processing tools
        self.bridge = CvBridge()
        self.image_height = 0
        self.image_width = 0
        self.mid_x = 0
        self.mid_y = 0
        self.human_cascade = self.initialize_model()
        # Flags to track the state of the drone
        self.target_found = False
        self.arrived_at_target = False
        self.broadcasting = False
        # GPS
        self.gps_sub = self.create_subscription(PointStamped, '/Mavic_2_PRO/gps', self.gps_callback, 10)
        self.origin_lat = None
        self.origin_lon = None
        self.current_pos = (0.0, 0.0)
        self.waypoints = []
        self.current_waypoint_index = 0
        self.tolerance = 1.0 # how many meters away from a waypoint to consider it found
        self.step = 5.0  # meters
        # Publisher for broadcasting GPS coordinates when target is found
        self.gps_publisher = self.create_publisher(PointStamped, '/target/gps', 10)
        self.target_publisher = self.create_publisher(String, '/target/found', 10)
        # Publisher for the drone's velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/Mavic_2_PRO/cmd_vel', 10)

    '''
    GPS and navigation
    '''

    def gps_callback(self, msg):
        if self.origin_lat is None or self.origin_lon is None:
            # Set the origin coordinates from the first GPS message
            self.origin_lat = msg.point.x
            self.origin_lon = msg.point.y
            self.get_logger().info(f'Setting origin GPS coordinates: ({self.origin_lat}, {self.origin_lon})')
        # Update the current position
        self.current_pos = (msg.point.x, msg.point.y)
        if self.broadcasting:
            print(msg.point.x, msg.point.y, msg.point.z)
        self.navigate()  # Call the navigate method to control the drone's movement

    def generate_lawn_mower_path(self):
        self.get_logger().info('Generating waypoints.')
        # min and max values for x and y coordinates
        xmin, xmax = 0.0, 50.0
        ymin, ymax = 0.0, 50.0
        forward = True
        y = ymin
        while y <= ymax:
            x_range = (xmin, xmax) if forward else (xmax, xmin)
            xs = self.linspace(*x_range, 5)
            for x in xs:
                # Convert waypoint (x, y) from local (meters) to world (GPS) coordinates
                lat = self.origin_lat + x
                lon = self.origin_lon + y
                self.waypoints.append((lat, lon))
            y += self.step
            forward = not forward

    def linspace(self, start, end, num):
        # Generate a list of evenly spaced numbers over a specified interval.
        if num == 1:
            return [start]
        step = (end - start) / (num - 1)
        return [start + i * step for i in range(num)]

    def navigate(self):
        '''
        Navigation in a lawn mover pattern. Maybe change to outwards spiral or something
        '''
        #self.get_logger().info('Navigating to the next waypoint.')
        # Generate waypoints in a lawn mower pattern if not already generated
        if not self.waypoints:
            self.generate_lawn_mower_path()
        
        # If there are waypoints left, navigate to the next one
        if self.current_waypoint_index < len(self.waypoints):
            #self.get_logger().info(f'Current waypoint index: {self.current_waypoint_index}')
            # Get the current position from the GPS message
            if self.origin_lat is None or self.origin_lon is None:
                self.get_logger().warn('Origin GPS coordinates are not set. Cannot navigate.')
                return
            # Calculate the current position in meters from the origin
            current_x = self.current_pos[0] - self.origin_lat
            current_y = self.current_pos[1] - self.origin_lon
            # Check if the drone is close enough to the current waypoint
            target_x, target_y = self.waypoints[self.current_waypoint_index]
            current_x, current_y = self.current_pos
            distance_x, distance_y = target_x - current_x, target_y - current_y
            dsitance = math.hypot(distance_x, distance_y)
            if dsitance < self.tolerance:
                self.get_logger().info(f'Arrived at waypoint {self.current_waypoint_index + 1}, coordinates {self.waypoints[self.current_waypoint_index]}.')
                self.get_logger().info(f'Own coordinates {self.current_pos}, next waypointt {self.waypoints[self.current_waypoint_index + 1]}.')
                self.current_waypoint_index += 1
                return  # Move to the next waypoint
            if dsitance > 0 or dsitance < 0:
                # Normalize direction and send velocity
                '''
                Now it just sends a velocity command but it's not to the target waypoint
                '''
                cmd = Twist()
                cmd.linear.x = distance_x / dsitance
                cmd.linear.y = distance_y / dsitance
                self.cmd_vel_publisher.publish(cmd)
        else:
            self.get_logger().info('All waypoints have been visited without finding the target.')

    '''
    Image processing and target detection
    '''

    def listener_callback(self, msg):
        try:
            # Get the image from the drone camera
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # If image dimensions are not set, calculate them from the first image
            if self.mid_x == 0 and self.mid_y == 0:
                height, width, _ = cv_image.shape
                self.image_height = height
                self.image_width = width
                self.mid_x = width // 2
                self.mid_y = height // 2
            
            # If terget is not found, fly around
            if not self.target_found:
                self.count_humans(cv_image)  # Detect humans in the image

            # If target is found, fly closer to it
            if self.target_found and not self.arrived_at_target:
                msg = String()
                msg.data = "true"
                self.target_publisher.publish(msg)
                self.fly_towards_human(cv_image)

            # If target is found and drone has arrived at the target, broadcast the location
            if self.arrived_at_target:
                self.broadcast_location()

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def initialize_model(self):
        # Use the local haarcascade_fullbody.xml from the package's share directory
        package_share_directory = ament_index_python.get_package_share_directory('my_package')
        haarcascade_path = os.path.join(package_share_directory, 'data', 'haarcascade_fullbody.xml')
        return cv2.CascadeClassifier(haarcascade_path)

    def send_random_command(self):
        msg = Twist()
        msg.linear.x = random.uniform(-1.0, 1.0)
        msg.angular.z = random.uniform(-0.5, 0.5)
        self.cmd_vel_publisher.publish(msg)

    def count_humans(self, image):
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Detect humans in the image
        humans = self.human_cascade.detectMultiScale(gray, 1.1, 4)
        if len(humans) > 0 and not self.target_found:
            self.get_logger().info(f'Detected {len(humans)} human(s) in the image.')
            self.target_found = True
        return humans
            
    def fly_towards_human(self, image):
        humans = self.count_humans(image)
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

            # Check if the bounding box height or width is at least 80% of the image height
            if h >= 0.8 * self.image_height or w >= 0.8 * self.image_height:
                self.arrived_at_target = True
                self.get_logger().info('Arrived at target: bounding box height >= 80% of image height.')

            # Fly towards the center of the detected human
            self.match_midpoints(x + w // 2, y + h // 2)

    def match_midpoints(self, target_x, target_y):
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

    def broadcast_location(self):
        if not self.broadcasting:
            self.broadcasting = True
            self.get_logger().info('Broadcasting target location.')

    '''
    Drone control methods
    '''

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info('Published stop command to /cmd_vel.')


    def move_forward(self):
        msg = Twist()
        msg.linear.x = 1.0  # Move forward
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info('Published forward velocity command to /cmd_vel.')
        time.sleep(1.5) # Stabilization delay
        self.stop()  # Stop after moving

    def turn_left(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 1.0
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info('Published left turn command to /cmd_vel.')
        time.sleep(1.5) # Stabilization delay
        self.stop()  # Stop after turning

    def turn_right(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -1.0
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info('Published right turn command to /cmd_vel.')
        time.sleep(1.5) # Stabilization delay
        self.stop()  # Stop after turning

    def change_altitude(self, altitude):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = altitude
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f'Published altitude change command to /cmd_vel with altitude {altitude}.')
        time.sleep(1.5) # Stabilization delay
        self.stop()  # Stop 

    def fly_around(self):
        return
        # This method will contain logic to control the drone's flight
        # For now, we will just log that the drone is flying around
        #self.get_logger().info('Mavic 2 Pro is flying around.')

        #self.send_random_command()  # Send a random command to the drone
        #self.move_forward()  # Move forward as a default action


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