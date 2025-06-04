# turtlebot_controller/turtlebot_controller.py
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

        # Turtlebot internal state variables:
        self.current_pos = (0.0, 0.0) # Turtlebot location
        self.current_rotation = 0.0   # Turtlebot rotation

        self.target_found = "False"   # Information from the drone: whether the target location has been found
        self.target_pose = (0.0, 0.0) # Target location from the drone
        
        self.lock_target = False      # Lock the target location for the turtlebot

        self.robot_moving = False     # Information whether the turtlebot is moving to the target location 
        self.orientation_ok = False   # Information whether the turtlebot orientation to the target is aligned

        # Turtlebot state broadcaster
        self.state_publisher = self.create_publisher(String, '/turtlebot/state', 10)

        # Control loop for turtlebot movement
        self.publisher_ = self.create_publisher(TwistStamped, '/turtlebot/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Subscribers to the target location information from the drone
        self.target_sub = self.create_subscription(Pose, '/target/pose', self.target_callback, 10)
        self.target_found_sub = self.create_subscription(String, '/target/found', self.target_found_callback, 10)

        # Subscribers to the turtlebot location: accurate position with GPS and orientation with odometry
        self.location_subscriber = self.create_subscription(PointStamped, '/TurtleBot3Burger/gps', self.gps_callback, 10)
        self.orientation_subscriber = self.create_subscription(Odometry, '/turtlebot/odom', self.odom_callback, 10)

        
    def target_callback(self, msg):
        # Update the target location for turtlebot ONLY when the drone confirms the target AND when the turtlebot does not yet have the target
        # location locked
        if self.target_found.lower() == 'true' and self.lock_target == False:
            self.target_pose = msg.position.x, msg.position.y
            self.get_logger().info(f'Target location received: {self.target_pose}')
            # Lock the target location to prevent the target location drifting for turtlebot navigation
            self.lock_target = True 
    
    def target_found_callback(self, msg):
        self.target_found = msg.data
        if not self.robot_moving:
            self.get_logger().info(f'Target found status: {self.target_found}')
        # Set turtlebot to "starting to move to location"-mode when the drone confirms the target has been found AND when the turtlebot is not yet in the particular mode
        if self.target_found.lower() == 'true' and self.robot_moving == False:
            self.get_logger().info('Target location confirmed')
            # Broadcast the information of turtlebot starting the process of moving to target location
            msg_turtlebot = String()
            msg_turtlebot.data = "starting"
            self.state_publisher.publish(msg_turtlebot)
            # Set the robot to "moving to location"-mode to allow navigation
            self.robot_moving = True
    
    def gps_callback(self, msg):
        # Get only the current position from GPS (X, Y-coordinates) to avoid redundant information storage
        self.current_pos = (msg.point.x, msg.point.y)

    def odom_callback(self, msg):
        # Get only the orientation as the position with GPS is more accurate than drifting odometry
        orientation = msg.pose.pose.orientation

        # Convert quaternion to Euler angles
        # Yaw is the rotation in z-axis, which is the only one needed to define orientation for navigation
        (_, _, self.current_rotation) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    
    def angle_difference(self, target, current):
        # Compute shortest angle difference between target and current rotation (in radians)
        # I.e. change the difference to be between -pi and pi
        diff = target - current
        while diff > math.pi:
            diff = diff - 2 * math.pi
        while diff < -math.pi:
            diff = diff + 2 * math.pi
        return diff
    
    def timer_callback(self):
        # Move the robot only when the target location has been found and confirmed
        if self.robot_moving and self.lock_target:
            self.get_logger().info(f'Moving to target location: {self.target_pose}')
            # Define the X, Y coordinates for target and current location to avoid oscillation during navigation
            target_x, target_y = self.target_pose
            current_x, current_y = self.current_pos

            # Distance calculation to destination
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.hypot(dx,dy)

            # Orientation calculation compared to destination
            target_yaw = math.atan2(dy, dx)    # value between -pi and pi (radians)
            yaw_error = self.angle_difference(target_yaw, self.current_rotation) # needed change (between -pi to pi radians) in orientation to face towards destination

            msg = TwistStamped()

            # Orientation to the destination must be aligned before moving forwards to it
            # Error in orientation sufficient to move enough towards the target is 0.05 radians
            if (abs(yaw_error) > 0.05):
                self.orientation_ok = False
            else:
                self.orientation_ok = True

            # If distance to the target location is close enough, stop to avoid collision with the poor human
            if distance < 1.0:
                # Broadcast arrival to the drone
                self.publisher_.publish(msg)
                msg_turtlebot = String()
                msg_turtlebot.data = "arrived"

                # Stop the turtlebot
                self.state_publisher.publish(msg_turtlebot)
                return

            # Broadcast the current state of the turtlebot: it is still on its way to the destination
            msg_turtlebot = String()
            msg_turtlebot.data = "coming"
            self.state_publisher.publish(msg_turtlebot)

            # If orientation is not towards the target: ROTATE
            if not self.orientation_ok:  
                # Turn based on the sign and amount of yaw error. In case of big orientation error, it is ok to turn more. Smaller orientation error requires only
                # finetuning of the orientation.
                msg.twist.angular.z = 0.2 * yaw_error
                self.publisher_.publish(msg)
                time.sleep(1) # stabilization time of command taking place

                # Stop twisting for more stable odometry reading
                msg.twist.angular.z = 0.0
                self.publisher_.publish(msg)

            # If orientation is towards the target: MOVE FORWARD
            else:
                # If distance to destination is large, move forward with fast but stable speed. When distance becomes small, movement is slowed down to prevent
                # collision with the poor human.
                msg.twist.linear.x = min(4.0, distance)
                self.publisher_.publish(msg)
                time.sleep(1) # stabilization time for command to take place
                
        # Stay still, if there is no confirmed target location to go to
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