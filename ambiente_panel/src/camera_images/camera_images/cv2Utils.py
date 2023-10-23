import cv2
import numpy as np


def draw_grid(image,width, height, 
              rows = 5, 
              cols = 5, 
              color = (0, 255, 255), 
              thickness = 1):
    
    # Dibujar líneas horizontales
    for i in range(1, rows):
        y = int(i * height / rows)
        cv2.line(image, (0, y), (width, y), color, thickness)

    # Dibujar líneas verticales
    for i in range(1, cols):
        x = int(i * width / cols)
        cv2.line(image, (x, 0), (x, height), color, thickness)

def draw_square_with_center(image, center, r,
                            color = (255, 0, 255), 
                            thickness = 1):

    top  = (center[0] - r , center[1] - r)
    down = (center[0] + r , center[1] + r)
    cv2.rectangle(image, top, down, color, thickness)
