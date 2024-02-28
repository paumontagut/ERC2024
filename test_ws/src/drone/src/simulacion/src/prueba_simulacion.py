#!/usr/bin/env python3.8

import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

x = 0
y = 0
z = 0


def image_callback(msg):
    print("Llamada a Callback ______                                  LC")
    try:
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    except CvBridgeError as e:
        print(e)
    else:
        cv2.imshow('cv_img', image_np)
        cv2.waitKey(5)

def pose_callback(msg):
     print(msg)
     global x, y, z
     x = msg.position.x
     y = msg.position.y
     z = msg.position.z


def main():

    global x, y , z

    rospy.init_node('Prueba_drone')
    print(".. Nodo inicializado ...................................... NI")

    image_topic = '/bebop2/camera_base/image_raw/compressed'
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    print("Nodo suscrito a . . . .                                     NS")
    print("\t\t", image_topic)

    pose_topic = '/bebop2/odometry_sensor1/pose'
    rospy.Subscriber(pose_topic, Pose, pose_callback)
    print("Nodo suscrito a . . . .                                     NS")
    print("\t\t", pose_topic)

    print("Nodo publica a .. .. ..                                     NP")

    takeof_topic = '/bebop/takeoff'
    pub_takeof = rospy.Publisher(takeof_topic, Empty, queue_size=10)
    print("\t\t", takeof_topic)

    land_topic = '/bebop/land'
    pub_land = rospy.Publisher(land_topic, Empty, queue_size=10)
    print("\t\t", land_topic)

    vel_topic = '/bebop/cmd_vel'
    pub_vel = rospy.Publisher(vel_topic, Twist, queue_size=10)
    print("\t\t", vel_topic)
    
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

         # Inicializar un mensaje Empty
        takeoff_msg = Empty()

         # Publicar el mensaje una sola vez
        pub_takeof.publish(takeoff_msg)

        # Esperar un período de 3 seg 
        rospy.sleep(3)

        twist_msg = Twist()

        while (z < 5):
            twist_msg.linear.z = 0.5 

            # Publicar el mensaje Twist
            pub_vel.publish(twist_msg)
            rospy.sleep(0.2)

            rate.sleep()
        





        rate.sleep()

    

if __name__ == "__main__":
    main()