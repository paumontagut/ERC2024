#!/usr/bin/env python3

# IMPORTACIONES
import rospy
import cv2
import numpy as np
import math
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialArray
from time import sleep

# CLASE DEL PROGRAMA
class Ceabot:
    # AJUSTES INICIALES
    
    def __init__(self):
        rospy.init_node("ceabot")
        self.sub = rospy.Subscriber("/fiducial_vertices", FiducialArray, self.read)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.move = Twist()
        
        self.lado = 0

    # INICIAR
    def inicio(self):
        rospy.spin()
        
        
    # PROGRAMA DE DETECCIÓN Y SEGUIMIENTO
    def read(self, fiducials):
        try:
            # ASIGNACIÓN DE PARÁMETROS
                    """laser_data = LaserDatam()
                    laser_data.recoger_datos(laser_data)
                    max1 = laser_data.max1
                    max2 = laser_data.max2
                    max3 = laser_data.max3
                    max4 = laser_data.max4
                    
                    print(max1, max2, max3, max4)"""
                    
                    vertices = fiducials.fiducials[0]

                    dif_vertices = max(vertices.y3 - vertices.y0, vertices.y2 - vertices.y1)
                
                    if dif_vertices > 75:  # CERCA
                        self.move.linear.x = 0.00
                
                    elif dif_vertices > 30:  # DISTANCIA MEDIA
                        self.move.linear.x = 0.18
                 
                    elif dif_vertices <= 30:  # LEJOS
                        self.move.linear.x = 0.22
                 
                    if vertices.x0 < 80:
                        self.move.angular.z = 2.3
                        self.lado = 1

                    elif vertices.x0 <= 150: # POSICIONADO A LA IZQQUIERDA
                        self.move.angular.z = 1
                        self.lado = 1
 
                    elif vertices.x0 > 320:
                        self.move.angular.z = -2.3
                        self.lado = -1

                    elif vertices.x0 >= 250: # POSICIONADO A LA DERECHA
                        self.move.angular.z = -1
                        self.lado = -1

                    else: # CENTRADO
                        self.move.angular.z = 0
                        
                    
                    self.pub.publish(self.move)
                

        except IndexError:
        
            
            if self.lado == 1:
                self.move.linear.x = 0
                self.move.angular.z = 1
            elif self.lado == -1:
                self.move.linear.x = 0
                self.move.angular.z = -1
            
            
            
            self.pub.publish(self.move)

            
        
if __name__ == "__main__":


    nodo = Ceabot()
    nodo.inicio()