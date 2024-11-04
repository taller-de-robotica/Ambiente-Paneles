#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2

class ImageListener(Node):

    def __init__(self):
        super().__init__('image_listener')
        
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Muestra la imagen
        cv2.imshow("Image from Gazebo simulator", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    node = ImageListener()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
