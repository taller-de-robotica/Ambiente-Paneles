import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class RandomTwistPublisher(Node):

    def __init__(self):
        super().__init__('random_twist_publisher')
        self.publisher_ = self.create_publisher(Twist, 'commands/velocity', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = random.uniform(-1.0, 1.0)
        msg.linear.y = random.uniform(-1.0, 1.0)
        msg.linear.z = random.uniform(-1.0, 1.0)
        msg.angular.x = random.uniform(-1.0, 1.0)
        msg.angular.y = random.uniform(-1.0, 1.0)
        msg.angular.z = random.uniform(-1.0, 1.0)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)

    node = RandomTwistPublisher()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
