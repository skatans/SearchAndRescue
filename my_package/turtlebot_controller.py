# turtlebot_controller/simple_controller.py
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion
import math
import time


class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        # Internal state
        self.target_pose = (0.0, 0.0) # Target location from the drone
        self.target_found = "False"   # Information whether to go to the target location or not
        self.robot_moving = False     # Information whether the turtlebot is moving to the location 
        self.current_pos = (0.0, 0.0) # Turtlebot position
        self.current_rotation = 0.0   # Turtlebot rotation
        self.orientation_ok = False   # Turtlebot orientation to the target (is the robot aligned)
        self.lock_target = False      # Lock the target location when the drone says it is found

        # Turtlebot state broadcaster
        self.state_publisher = self.create_publisher(String, '/turtlebot/state', 10)

        # Control loop for movement
        self.publisher_ = self.create_publisher(TwistStamped, '/turtlebot/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Subscribers to the drone location
        self.target_sub = self.create_subscription(Pose, '/target/pose', self.target_callback, 10)
        self.target_found_sub = self.create_subscription(String, '/target/found', self.target_found_callback, 10)

        # Subscribers to the turtlebot location
        self.location_subscriber = self.create_subscription(PointStamped, '/TurtleBot3Burger/gps', self.gps_callback, 10)
        self.orientation_subscriber = self.create_subscription(Odometry, '/turtlebot/odom', self.odom_callback, 10)

        

    def target_callback(self, msg):
        if self.target_found.lower() == 'true' and self.lock_target == False:
            self.target_pose = msg.position.x, msg.position.y
            self.lock_target = True
    
    def target_found_callback(self, msg):
        self.target_found = msg.data
        if self.target_found.lower() == 'true' and self.robot_moving == False:
            msg_turtlebot = String()
            msg_turtlebot.data = "Start moving to location"
            self.state_publisher.publish(msg_turtlebot)
            self.robot_moving = True
    
    def gps_callback(self, msg):
        self.current_pos = (msg.point.x, msg.point.y)

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation

        (_, _, self.current_rotation) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    
    def angle_difference(self, target, current):
        # Compute shortest angle difference between target and current rotation (radians)
        # I.e. change it to be between -pi and pi
        diff = target - current
        while diff > math.pi:
            diff = diff - 2 * math.pi
        while diff < -math.pi:
            diff = diff + 2 * math.pi
        return diff
    
    def timer_callback(self):
        if self.robot_moving and self.lock_target: # Move only when the target has been found
            target_x, target_y = self.target_pose
            current_x, current_y = self.current_pos

            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.hypot(dx,dy)

            target_yaw = math.atan2(dy, dx) # value between -pi and pi (radians)
            yaw_error = self.angle_difference(target_yaw, self.current_rotation)

            msg = TwistStamped()

            # Orientation to the target must be aligned before moving to the target  
            if (abs(yaw_error) > 0.05):
                self.orientation_ok = False
            else:
                self.orientation_ok = True

            # If distance to the target is sufficient, no need to move anymore
            if distance < 1.0:
                self.publisher_.publish(msg)
                msg_turtlebot = String()
                msg_turtlebot.data = "Arrived at the location"
                self.state_publisher.publish(msg_turtlebot)
                return

            msg_turtlebot = String()
            msg_turtlebot.data = "Moving to location"
            self.state_publisher.publish(msg_turtlebot)

            # If orientation is not ok TURN
            if not self.orientation_ok:  
                # Turn based on the sign and amount of yaw error
                msg.twist.angular.z = 0.2 * yaw_error
                self.publisher_.publish(msg)
                time.sleep(1)
                msg.twist.angular.z = 0.0
                self.publisher_.publish(msg)
            # If orientation is ok MOVE FORWARD
            else:
                msg.twist.linear.x = min(4.0, distance)
                self.publisher_.publish(msg)
                time.sleep(1)
            
        else:
            msg = TwistStamped()
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()