#! /usr/bin/python3

# La mayoría de estos importos son de ROS, no serán necesarios
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from transforms3d.euler import mat2euler
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
from cv2 import aruco
import numpy as np

#Estas 4 líneas eran para enviar los mensajes a otro programa tampoco se necesitan de momento.
list_latdistance = Float32MultiArray()
list_frontdistance = Float32MultiArray()
list_orientation = Float32MultiArray()
parameters = Float32MultiArray()

frame = 0
bridge = CvBridge()   # Para poder pasar la imagen del formato de ros a otros formatos

"""def callback(msg):
    global end_program
    print("Objetivo de distancia alcanzado")
    latdistance_pub.publish(list_latdistance)
    frontdistance_pub.publish(list_frontdistance)
    end_program = msg.data"""

# Callback cada vez que se actualiza la imagen
def image_callback(img_msg):
    global frame, bridge
    try:
        frame = bridge.compressed_imgmsg_to_cv2(img_msg)   # Pasar de imagen comprimida de ROS a imagen OpenCV
        frame = cv.rotate(frame, cv.ROTATE_180)  # Rotar la imagen porque estaba mal puesta en el robot
    except CvBridgeError as e:   # Ver problemas al transformar
        print(e)
        return

    # Camra matrix y dist_coefs son valores que obtuve del programa para calibrar la cámara y estimar lo mejor posible
    # los valores de distancias y ángulos.
    camera_matrix = np.array([[571.20712307, 0, 407.87169985],
                     [0, 571.55029023, 302.6283221],
                     [0, 0, 1]])

    dist_coefs = np.array([[-0.02036823, 0.0603381, -0.00571633, -0.01340911, -0.08042632]])

    marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)  # Diccionario del tipo de aruco que tenía que detectar
    param_markers = aruco.DetectorParameters()
    marker_size = 0.12 # 12 #5   #Tamaño del aruco en metros

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)   # Pasar la imagen a tonos de grises, más rápido
    marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)

    if marker_corners:  # Si hemos detectado algún aruco.
        rospy.loginfo('Encontrado')

        #Estimar distancias y orientaciones
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, marker_size, camera_matrix, dist_coefs)
        total_markers = range(0, marker_IDs.size)  # Número de markers
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()

            poit = cv.drawFrameAxes(frame, camera_matrix, dist_coefs, rVec[i], tVec[i], 3, 2)
            cv.putText(frame, f"id: {ids[0]} dfront: {round(tVec[i][0][2], 2)}  dlat: {round(tVec[i][0][0], 2)}",
                       top_right, cv.FONT_HERSHEY_PLAIN, 1.3, (200, 100, 0), 2, cv.LINE_AA)

            # Lo de arriba es lo que se va a ver en la imagen final como que estamos detectando

            R_marker, _ = cv.Rodrigues(rVec[i])
            roll_global, pitch_global, yaw_global = mat2euler(R_marker, 'sxyz')  # Obtener los ángulos
            yaw_global_deg = np.degrees(yaw_global)
            roll_global_deg = np.degrees(roll_global)
            pitch_global_deg = np.degrees(pitch_global)
            lateral_distance = round(tVec[i][0][0],2)
            frontal_distance = round(tVec[i][0][2], 2)
            altitude_distance = round(tVec[i][0][1], 2)

            #Enviar por mensaje de ROS lo que necesitaba
            parameters.data = [lateral_distance, altitude_distance, frontal_distance, roll_global_deg, pitch_global_deg, yaw_global_deg]
            parameters_pub.publish(parameters)
            #list_latdistance.data.append(round(tVec[i][0][0], 2))
            #list_frontdistance.data.append(round(tVec[i][0][2], 2))
            #list_orientation.data.append(round(yaw_global_deg, 2))

    cv.imshow('frame', frame)  # Mostrar la imagen alterada
    key = cv.waitKey(1)



if __name__ == '__main__':
    rospy.init_node('aruco_markers_detection')
    #latdistance_pub = rospy.Publisher('/aruco/lateral_distance', Float32MultiArray, queue_size=10)
    #frontdistance_pub = rospy.Publisher('/aruco/front_distance', Float32MultiArray, queue_size=10)
    #orientation_pub = rospy.Publisher('/aruco/orientation', Float32MultiArray, queue_size=10)
    parameters_pub = rospy.Publisher('/aruco/parameters', Float32MultiArray, queue_size=10)
    #distobjective_sub = rospy.Subscriber('aruco/objective_distance', Bool, callback)
    image_sub = rospy.Subscriber('compressed_image', CompressedImage, image_callback)

    try:
        while not rospy.is_shutdown():
            rospy.spin()
    except KeyboardInterrupt or rospy.ROSInterruptException:
            cv.destroyAllWindows()


