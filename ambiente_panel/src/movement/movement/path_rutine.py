import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random
import time

class RandomTwistPublisher(Node):

    def __init__(self):
        super().__init__('random_twist_publisher')
        self.publisher_ = self.create_publisher(Twist, 'commands/velocity', 10)
        timer_period = 1.0  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.make_rutine()
    
    def make_rutine(self):

        # self.rotate(360)
        for _ in range(5):
            self.rotate(90,-1)
            time.sleep(0.5)
            self.rotate(90,1)
            self.moveX(2)
            time.sleep(1.0)


    def moveX(self, distance, direction = 1):
        msg = Twist()
        msg.linear.x = 1.0 * direction
        self.publish_mesagge(msg)

        time.sleep(distance)
        msg = Twist()
        msg.linear.x = 0.0
        self.publish_mesagge(msg)

    def rotate(self, degrees, direction = 1):
        
        msg = Twist()
        msg.angular.z = 1.0 * direction
        self.publish_mesagge(msg)
        time.sleep(degrees / 45)
        msg = Twist()
        msg.angular.z  = 0.0
        self.publish_mesagge(msg)

    def publish_mesagge(self, msg):
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
