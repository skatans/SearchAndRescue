# turtlebot_controller/simple_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = TwistStamped()
        msg.twist.linear.x = 0.2
        msg.twist.angular.z = 0.3
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing cmd_vel: linear=0.2, angular=0.3')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
