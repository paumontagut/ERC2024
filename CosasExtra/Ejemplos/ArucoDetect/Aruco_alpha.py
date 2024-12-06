#!/usr/bin/env python3

# IMPORTACIONES
import rospy
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialArray
from time import sleep

# CLASE DEL PROGRAMA
class Ceabot:

	# AJUSTES INICIALES
    def __init__(self):
        rospy.init_node("ceabot")
        self.sub = rospy.Subscriber("/fiducial_vertices",FiducialArray, self.read)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.move = Twist()
    
	# INICIAR
    def inicio(self):
        rospy.spin()
    
	# PROGRAMA DE DETECCIÓN Y SEGUIMIENTO
    def read(self, fiducials):
        try:
        	# ASIGNACIÓN DE PARÁMETROS
            vertices = fiducials.fiducials[0]
            dif_vertices = vertices.x1 - vertices.x0
            # ASIGNACIÓN DE LA VELOCIDAD LINEAL DEPENDIENDO DE LO CERCA/LEJOS QUE
        	# ESTÉ EL ARUCO
            if dif_vertices > 80: # CERCA
                self.move.linear.x = 0.05
           	 
            elif dif_vertices <= 80 and dif_vertices > 30: # DISTANCIA MEDIA
                self.move.linear.x = 0.15
       	 
            else: # LEJOS
                self.move.linear.x = 0.5
       	 
        	# ASIGNACIÓN DE LA VELOCIDAD ANGULAR (GIRO) DEPENDIENDO DE LA POSICIÓN
        	# DEL ARUCO EN LA CÁMARA
            if vertices.x0 <= 125: # POSICIONADO A LA IZQQUIERDA
                self.move.angular.z = 1
       	    
            elif vertices.x0 >= 250: # POSICIONADO A LA DERECHA
                self.move.angular.z = -1
           	 
            else: # CENTRADO
                self.move.angular.z = 0

            self.pub.publish(self.move)
   	 
        except IndexError:
            pass

if __name__ == "__main__":
	nodo = Ceabot()
	nodo.inicio()
