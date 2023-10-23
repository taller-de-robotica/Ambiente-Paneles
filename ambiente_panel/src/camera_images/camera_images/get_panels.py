#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from .cv2Utils import draw_grid, draw_square_with_center

class ImageListener(Node):

    def __init__(self):
        super().__init__('image_listener')
        
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, 'commands/velocity', 10)
        self.subscription  # Prevent unused variable warning
  
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Muestra la imagen
        self.get_panel(cv_image)
    
    def get_panel(self, image):
        final_cotours = []
        alto, ancho = image.shape[:2]

        # Convertir a escala de grises
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # filtro gaussiano para reducir el ruido
        blurred_image = cv2.GaussianBlur(gray,(3,3),cv2.BORDER_DEFAULT)
        # Binarizar la imagen (puedes ajustar los valores según tu caso)

        _, binary_image = cv2.threshold(blurred_image, 100, 255, cv2.THRESH_BINARY)
        # _, binary_image = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
        # Encontrar contornos en la imagen binarizada
        contours, hierarchy = cv2.findContours(binary_image,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for i,contour in enumerate(contours):
            epsilon = 0.02 * cv2.arcLength(contour, True)  # Puedes ajustar este valor
            simplified_contour = cv2.approxPolyDP(contour, epsilon, True)
            if len(simplified_contour) == 4:
                final_cotours.append(simplified_contour)
                cv2.drawContours(image, [simplified_contour], 0, (0, 0, 255), 3)

        hit = False
        left_detector_center = (45,125)
        right_detector_center = (275,125)
        radius = 35

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
        return final_cotours


def hit_detector(point, detector_center, radius):
    if point[0] > detector_center[0] + radius or point[0] < detector_center[0] - radius:
        return False
    if point[1] > detector_center[1] + radius or point[1] < detector_center[1] - radius:
        return False
    return True

def main(args=None):
    print('HELLOOOO')
    rclpy.init(args=args)

    node = ImageListener()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()