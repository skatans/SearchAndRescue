import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from tf_transformations import euler_from_quaternion

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
        self.initialized = False
        self.target_found = False
        self.arrived_at_target = False
        self.broadcasting = False
        # IMU
        self.imu_sub = self.create_subscription(Imu, '/Mavic_2_PRO/imu', self.imu_callback, 10)
        self.yaw = 0.0
        # GPS
        self.gps_sub = self.create_subscription(PointStamped, '/Mavic_2_PRO/gps', self.gps_callback, 10)
        self.origin_lat = None
        self.origin_lon = None
        self.current_pos = (0.0, 0.0)
        self.waypoints = []
        self.current_waypoint_index = 0
        self.tolerance = 5.0 # how many meters away from a waypoint to consider it found
        self.step = 20.0  # meters
        # Publisher for broadcasting GPS coordinates when target is found
        self.pose_publisher = self.create_publisher(Pose, '/target/pose', 10)
        self.target_publisher = self.create_publisher(String, '/target/found', 10)
        # Publisher for the drone's velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/Mavic_2_PRO/cmd_vel', 10)

    '''
    IMU, GPS and navigation
    '''

    def imu_callback(self, msg):
        # Process IMU data
        # z is the yaw, x is the roll, y is the pitch
        # yaw is where the drone is facing, roll is the tilt to the left or right, pitch is the tilt forward or backward
        #self.get_logger().info(f'Orientation: {msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}, {msg.orientation.w}')
        # Convert quaternion to Euler angles
        orientation_q = msg.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.yaw = yaw  # The heading relative to north (in radians)
        #self.get_logger().info(f'Yaw (heading): {yaw}')

    def gps_callback(self, msg):
        #self.get_logger().info(f'Received GPS coordinates: ({msg.point.x}, {msg.point.y})')
        if not self.initialized:
            # Set the origin coordinates from the first GPS message
            self.origin_lat = msg.point.x
            self.origin_lon = msg.point.y
            self.get_logger().info(f'Setting origin GPS coordinates: ({self.origin_lat}, {self.origin_lon})')
            # Send a command to rise up
            self.change_altitude(5.0)
            time.sleep(5)  # Wait for some seconds to rise up
            self.stop()
            self.initialized = True
        # Update the current position
        self.current_pos = (msg.point.x, msg.point.y)
        if not self.target_found:
            self.navigate()  # Call the navigate method to control the drone's movement

    def generate_lawn_mower_path(self):
        self.get_logger().info('Generating waypoints.')
        # min and max values for x and y coordinates
        xmin, xmax = 0.0, 100.0
        ymin, ymax = 0.0, 100.0
        forward = True
        y = ymin
        while y <= ymax:
            x_range = (xmin, xmax) if forward else (xmax, xmin)
            xs = self.linspace(*x_range, 10)  # Generate 10 waypoints along the x-axis
            for x in xs:
                # Convert waypoint (x, y) from local (meters) to world (GPS) coordinates
                lat = self.origin_lat + x
                lon = self.origin_lon + y
                self.waypoints.append((lat, lon))
            y += self.step # Move to the next row
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
            distance = math.hypot(distance_x, distance_y)
            if distance < self.tolerance:
                self.get_logger().info(f'Arrived at waypoint {self.current_waypoint_index + 1}, coordinates {self.waypoints[self.current_waypoint_index]}.')
                self.get_logger().info(f'Own coordinates {self.current_pos}, next waypointt {self.waypoints[self.current_waypoint_index + 1]}.')
                self.current_waypoint_index += 1
                return  # Move to the next waypoint
            else:
                # Calculate the desired angle to the next waypoint
                desired_yaw = math.atan2(distance_y, distance_x)
                # Calculate the difference between the current yaw and the desired yaw
                yaw_diff = desired_yaw - self.yaw
                # Normalize the angle to the range [-pi, pi]
                yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi

                # If the drone is not facing the waypoint (within 0.2 radians), turn towards it
                if abs(yaw_diff) > 0.2:
                    if yaw_diff > 0:
                        self.turn_left()
                    else:
                        self.turn_right()
                    time.sleep(1.5) # Stabilization delay
                    self.stop_turning()  # Stop after turning
                else:
                    self.move_forward(2.5)
                    time.sleep(5.0) # Stabilization delay
        else:
            self.get_logger().info('All waypoints have been visited without finding the target.')
            # Expand the search area by generating new waypoints around the original ones
            # Calculate the bounding box of the original waypoints
            lats = [wp[0] for wp in self.waypoints]
            lons = [wp[1] for wp in self.waypoints]
            lat_min, lat_max = min(lats), max(lats)
            lon_min, lon_max = min(lons), max(lons)
            expansion = self.step * 2  # Expand the search area by 2 steps
            # Generate new waypoints around the perimeter (clockwise)
            expanded = [
                (lat_min - expansion, lon_min - expansion),
                (lat_min - expansion, lon_max + expansion),
                (lat_max + expansion, lon_max + expansion),
                (lat_max + expansion, lon_min - expansion)
            ]
            self.waypoints.extend(expanded)
            self.get_logger().info(f'Added expanded waypoints: {expanded}')
            self.current_waypoint_index = len(self.waypoints) - len(expanded)

    '''
    Image processing and target detection
    '''

    def listener_callback(self, msg):
        if not self.initialized:
            return
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
                self.stop()  # Stop the drone before starting to fly towards the human
                time.sleep(1.5)  # Stabilization delay
            # Draw bounding boxes around the detected human
            for (x, y, w, h) in humans:
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Display the image with bounding boxes
            cv2.imshow("Mavic 2 Pro Detections", image)
            cv2.waitKey(1)

            # Check if the bounding box height or width is at least 70% of the image height
            if h >= 0.7 * self.image_height or w >= 0.7 * self.image_height:
                self.arrived_at_target = True
                self.get_logger().info('Arrived at target')
                self.stop()

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
            time.sleep(1.5) # Stabilization delay
            self.stop()  # Stop after moving
        elif abs(offset_y) > 50:
            if offset_y < 0:
                self.change_altitude(0.1)
            else:
                self.change_altitude(-0.1)
            time.sleep(1.5) # Stabilization delay
            self.stop()  # Stop after moving
        else:
            self.move_forward()
        time.sleep(1.5) # Stabilization delay

    def broadcast_location(self):
        if not self.broadcasting:
            self.broadcasting = True
            self.get_logger().info('Broadcasting target location.')
            self.stop()  # Send stop command in case the drone is moving

        # Pose for target location
        pose = Pose()

        # Get the current GPS coordinates to a Point
        point = Point()
        point.x = self.current_pos[0]  # Latitude
        point.y = self.current_pos[1]  # Longitude
        point.z = 0.0  # Altitude
        pose.position = point

        # Quaternion might not be necessary for 2D navigation for the turtlebot, but included for completeness
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(self.yaw / 2.0)
        quaternion.w = math.cos(self.yaw / 2.0)
        pose.orientation = quaternion

        self.pose_publisher.publish(pose)

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


    def stop_turning(self):
        msg = Twist()
        msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info('Published stop_turning command to /cmd_vel.')


    def move_forward(self, velocity=1.0):
        msg = Twist()
        msg.linear.x = velocity # Move forward
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info('Published forward velocity command to /cmd_vel.')

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 1.0
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info('Published left turn command to /cmd_vel.')

    def turn_right(self):
        msg = Twist()
        msg.angular.z = -1.0
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info('Published right turn command to /cmd_vel.')

    def change_altitude(self, altitude):
        msg = Twist()
        msg.linear.z = altitude
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f'Published altitude change command to /cmd_vel with altitude {altitude}.')


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