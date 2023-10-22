#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

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
        get_panel(cv_image)

        # cv2.imshow("Image from Ignition Gazebo", cv_image)
        # cv2.waitKey(1)

def main(args=None):
    print('HELLOOOO')
    rclpy.init(args=args)

    node = ImageListener()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# Cargar la imagen
# image = cv2.imread('panel_view.jpg')
def get_panel(image):
    final_cotours = []
    print(image.shape[:2])
    alto, ancho = image.shape[:2]
    total_area = alto * ancho
    print(total_area)
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
            area = cv2.contourArea(simplified_contour)
            print(simplified_contour)
            print(f'{i}.# Porcentaje del area ocupado {area/total_area}')
            cv2.drawContours(image, [simplified_contour], 0, (0, 0, 255), 3)

    # Mostrar la imagen con contornos
    blurred_image3 = cv2.cvtColor(blurred_image, cv2.COLOR_GRAY2BGR)
    binary_image3 = cv2.cvtColor(binary_image , cv2.COLOR_GRAY2BGR)
    resultado = np.hstack((blurred_image3,binary_image3, image))
    cv2.imshow('Contours', resultado)
    cv2.waitKey(1)
    return final_cotours