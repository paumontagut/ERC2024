

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


pattern_size = (8, 6)  # Número de esquinas interiores del patrón
square_size = 0.022  # Tamaño de cada lado de cada cuadro en el patrón de ajedrez en metros

# Crear una matriz para almacenar los puntos de calibración
obj_points = []  # Puntos 3D en el mundo real
img_points = []  # Puntos 2D en las imágenes

# Generar coordenadas 3D de los puntos de calibración
objp = np.zeros((np.prod(pattern_size), 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size
gray = 0
# Inicializar el nodo de ROS
rospy.init_node('camera_calibration')

# Crear un objeto CvBridge
bridge = CvBridge()

def image_callback(msg):
    global frame_counter, obj_points, img_points, gray

    # Convertir la imagen comprimida a una imagen OpenCV
    frame = bridge.compressed_imgmsg_to_cv2(msg)
    imagen_rot = cv2.rotate(frame, cv2.ROTATE_180)
    # Convertir a escala de grises
    gray = cv2.cvtColor(imagen_rot, cv2.COLOR_BGR2GRAY)

    cv2.imshow('Calibration', imagen_rot)
    key = cv2.waitKey(1)

    if key == 13:
        # Encontrar las esquinas del patrón de calibración
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if ret:
            # Si se encontraron las esquinas, agregar los puntos de calibración
            obj_points.append(objp)
            img_points.append(corners)

            # Dibujar y mostrar las esquinas en la imagen
            cv2.drawChessboardCorners(imagen_rot, pattern_size, corners, ret)
            cv2.imshow("hola", imagen_rot)
            print("Recibido")

        print(f"Frame capturado para calibración.")

    # Salir del bucle si se presiona la tecla 'q'
    if key == ord('q'):
        rospy.signal_shutdown("Calibración terminada")

# Suscribirse al topic de la imagen comprimida
rospy.Subscriber('/compressed_image', CompressedImage, image_callback)

# Iniciar la captura de imágenes
while not rospy.is_shutdown():
    rospy.spin()

# Calibrar la cámara
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

# Imprimir los parámetros de la cámara
print("Matriz de la cámara:")
print(camera_matrix)
print("\nCoeficientes de distorsión:")
print(dist_coeffs)

# Liberar los recursos
cv2.destroyAllWindows()

""""[[573.27885455   0.         408.94006952]
 [  0.         573.51994015 333.20938375]
 [  0.           0.           1.        ]]

Coeficientes de distorsión:
[[-0.07762187  0.3677111   0.00484313 -0.00365746 -0.62401102]]


Matriz de la cámara:
[[571.20712307   0.         407.87169985]
 [  0.         571.55029023 302.6283221 ]
 [  0.           0.           1.        ]]

Coeficientes de distorsión:
[[-0.02036823  0.0603381  -0.00571633 -0.01340911 -0.08042632]]"""


"""Matriz de la cámara:
[[572.79457073   0.         405.58010462]
 [  0.         572.06402503 308.59068408]
 [  0.           0.           1.        ]]

Coeficientes de distorsión:
[[-0.02095431  0.38842342 -0.00591757 -0.01294251 -1.08533934]]"""


