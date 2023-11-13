import rclpy

# For robot position 
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
from geometry_msgs.msg import Twist
# Camera
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import random
import time
import threading
import math
import cv2
import numpy as np

class RobotState:

    def __init__(self) -> None: 
        self.translate_x = 0
        self.translate_y = 0
        self.translate_z = 0

        self.rotate_x = 0
        self.rotate_y = 0
        self.rotate_z = 0
        self.w  = 0

    @property
    def deg_theta(self):
        return abs(2*math.acos(self.w))

    @property
    def deg_rotate_x(self):
        #TODO
        if self.deg_theta == 0:
            return 0
        return self.rotate_x / math.sin(self.deg_theta)
    
    @property
    def deg_rotate_y(self):
        #TODO https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        if self.deg_theta == 0:
            return 0
        return self.rotate_y / math.sin(self.deg_theta)
    
    @property
    def deg_rotate_z(self):
        siny_cosp = 2 * (self.w * self.rotate_z + self.rotate_x * self.rotate_y)
        cosy_cosp = 1 - 2 * (self.rotate_y **2 + self.rotate_z **2);        
        return math.atan2(siny_cosp, cosy_cosp) * 180 / math.pi
    
    def translate_magnitude(self):
        return math.sqrt(self.translate_x**2 + self.translate_y**2)
    
    def distance_to(self, x, y):
        return math.sqrt( (self.translate_x-x)**2 + (self.translate_y-y)**2)
    
    def update_state(self, msg):
        transform = msg.transforms[0].transform

        self.translate_x = transform.translation.x
        self.translate_y = transform.translation.y
        self.translate_z = transform.translation.z

        self.rotate_x = transform.rotation.x
        self.rotate_y = transform.rotation.y
        self.rotate_z = transform.rotation.z
        self.w  = transform.rotation.w

class PathMovement(Node):

    def __init__(self):
        super().__init__('random_twist_publisher')
        # Robot location
        self.position = RobotState()
        self.subscription_tf = self.create_subscription(
            TFMessage,
            '/model/kobuki_standalone/tf',
            self.position_callback,
            10)
        # Robot Camera
        self.bridge = CvBridge()
        self.subscription_camera = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10)
        self.panel_hit = False

        self.publisher_ = self.create_publisher(Twist, 'commands/velocity', 10)
        self.clear_buffer()
        time.sleep(4.0)
        print('Empezando rutina')
        self.routine = threading.Thread(target=self.make_rutine)
        self.routine.start()

    def position_callback(self, msg):
        self.position.update_state(msg)
        #print(f" grados rotados eje z {self.position.deg_rotate_z}", end='\r')
        # self.get_logger().info(f"Received TF2 transform: {self.position.rotate_z} ")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Muestra la imagen
        self.get_panel(cv_image)

    def get_panel(self, image):
        final_cotours = []

        # Convertir a escala de grises
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # filtro gaussiano para reducir el ruido
        blurred_image = cv2.GaussianBlur(gray,(3,3),cv2.BORDER_DEFAULT)
        # Binarizar la imagen 
        _, binary_image = cv2.threshold(blurred_image, 100, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(binary_image,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for i,contour in enumerate(contours):
            epsilon = 0.02 * cv2.arcLength(contour, True)  # tolerancia
            simplified_contour = cv2.approxPolyDP(contour, epsilon, True)
            if len(simplified_contour) == 4:
                final_cotours.append(simplified_contour)
                cv2.drawContours(image, [simplified_contour], 0, (0, 0, 255), 3)

        hit = False
        left_detector_center = (45,125)
        right_detector_center = (275,125)
        radius = 45

        # Detectar colision
        for contour in final_cotours:
            left_hits = 0
            right_hits = 0
            for point in contour:
                if hit_detector(point[0], left_detector_center, radius):
                    left_hits +=1
                if hit_detector(point[0], right_detector_center, radius):
                    right_hits +=1
            if left_hits == 1 and right_hits == 1:
                hit = True
                break

        # Dibujar detectores
        color = (0, 255,0)  if hit else (255, 0, 255) 
        draw_square_with_center(image, left_detector_center, radius, color=color)
        draw_square_with_center(image, right_detector_center, radius, color=color)

        # Mostrar la imagen con contornos
        blurred_image3 = cv2.cvtColor(blurred_image, cv2.COLOR_GRAY2BGR)
        binary_image3 = cv2.cvtColor(binary_image , cv2.COLOR_GRAY2BGR)
        resultado = np.hstack((blurred_image3,binary_image3, image))
        cv2.imshow('Contours', resultado)
        cv2.waitKey(1)
        if not self.panel_hit:
            self.panel_hit = hit 
    
    def make_rutine(self):
        
        self.move_m(9.0,velocity=2.0)
        time.sleep(0.4)
        self.rotate_d(90,velocity=0.5)
        time.sleep(0.4)
        self.move_m(3.9,velocity=1.0)
        time.sleep(0.4)
        self.rotate_d(90,velocity=0.5)
        time.sleep(0.4)
        self.move_m(1.8,velocity=2.0)

        for _ in range(9):
            time.sleep(0.4)
            d_to_panel = self.rotate_to_panel(direction=-1.0)
            print(f'Gire {360 - d_to_panel} grados hasta ver un panel')
            time.sleep(0.4)
            self.rotate_d(d_to_panel, direction=1, velocity=0.4)
            self.panel_hit = False
            time.sleep(0.4)
            self.move_m(1.8)

        time.sleep(0.4)
        self.rotate_d(90,velocity=0.5)
        time.sleep(0.4)
        self.move_m(4.0,velocity=2.0)
        time.sleep(0.4)
        self.rotate_d(90,velocity=0.5)
        time.sleep(0.4)
        self.move_m(1.4,velocity=1.0)

        for _ in range(9):
            time.sleep(0.4)
            d_to_panel = self.rotate_to_panel()
            print(f'Gire {d_to_panel} grados hasta ver un panel')
            time.sleep(0.4)
            self.rotate_d(d_to_panel, direction=-1.0, velocity=0.4)
            self.panel_hit = False
            time.sleep(0.4)
            self.move_m(1.8)


        time.sleep(0.4)
        self.rotate_d(90,velocity=0.5,direction=-1)
        time.sleep(0.4)
        self.move_m(3.7,velocity=2.0)
        time.sleep(0.4)
        self.rotate_d(90,velocity=0.5,direction=-1)
        time.sleep(0.4)
        self.move_m(1.2,velocity=1.0)

        for _ in range(9):
            time.sleep(0.4)
            d_to_panel = self.rotate_to_panel(direction=-1.0)
            print(f'Gire {360 - d_to_panel} grados hasta ver un panel')
            time.sleep(0.4)
            self.rotate_d(d_to_panel, direction=1, velocity=0.4)
            self.panel_hit = False
            time.sleep(0.4)
            self.move_m(1.8)
        
        

    def clear_buffer(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publish_messagge(msg)


    def move(self, velocity = 1.0, direction = 1):
        msg = Twist()
        msg.linear.x = velocity * direction
        self.publish_messagge(msg)


    def move_m(self, m, direction = 1, velocity = 1.0):
        p0 = (self.position.translate_x, self.position.translate_y)
        self.move(direction, velocity)
        d = self.position.distance_to(p0[0],p0[1])
        while d < m:
            d = self.position.distance_to(p0[0],p0[1])
            print(f'Distancia recorrida {d}' , end='\r')

        msg = Twist()
        msg.linear.x  = 0.0
        self.publish_messagge(msg)

    def rotate(self, direction = 1, velocity = 1.0):
        msg = Twist()
        msg.angular.z = velocity * direction
        self.publish_messagge(msg)


    def rotate_d(self, degrees, direction = 1, velocity = 1.0):
        z0 = self.position.deg_rotate_z
        acu = 0
        self.rotate(direction, velocity)
        while acu  < degrees:
            acu += abs(self.position.deg_rotate_z - z0)
            z0 = self.position.deg_rotate_z
            print(f'Grados girados {acu}' , end='\r')

        self.stop_rotation()

    def rotate_to_panel(self, direction = 1.0):
        z0 = self.position.deg_rotate_z 
        acu = 0
        self.rotate(velocity=0.5, direction=direction)
        while not self.panel_hit:
            pass
            # acu += abs(self.position.deg_rotate_z - z0)
            # z0 = self.position.deg_rotate_z
        acu = abs(self.position.deg_rotate_z  - z0)
        self.stop_rotation()
        return acu  
    
    def stop_rotation(self):
        msg = Twist()
        msg.angular.z  = 0.0
        self.publish_messagge(msg)

    def publish_messagge(self, msg):
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg)

def hit_detector(point, detector_center, radius):
    if point[0] > detector_center[0] + radius or point[0] < detector_center[0] - radius:
        return False
    if point[1] > detector_center[1] + radius or point[1] < detector_center[1] - radius:
        return False
    return True

def draw_square_with_center(image, center, r,
                            color = (255, 0, 255), 
                            thickness = 1):

    top  = (center[0] - r , center[1] - r)
    down = (center[0] + r , center[1] + r)
    cv2.rectangle(image, top, down, color, thickness)


def main(args=None):
    rclpy.init(args=args)

    node = PathMovement()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
