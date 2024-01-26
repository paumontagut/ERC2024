import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import cv2 
import os

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

class ArucoGenerate(Node):
    
    def __init__(self):
        super().__init__('aruco_generate')

        self.publisher_generate = self.create_publisher(String, 'aruco_generator_callback', 10) # Publisher for String message
        self.timer = self.create_timer(1, self.generate_callback)

    def generate_callback(self):
        
        aruco_type = "DICT_6x6_250"
        id = 1

        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

        print(f"ArUco type {aruco_type} with ID {id}")
        marker_image = cv2.aruco.generateImageMarker(arucoDict, id, 500) 

        current_dir = os.path.dirname(os.path.realpath(__file__))

        tag_name = current_dir + "test_images/" + aruco_type + "_" + str(id) + ".png"  # !!! Change the path to your image !!!
        cv2.imwrite(tag_name, marker_image)
        cv2.imshow("ArUco Tag", marker_image)

        msg = String()
        msg.data = tag_name + " generated"

        self.publisher_generate.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        cv2.waitKey(0)
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)

    aruco_generate = ArucoGenerate()

    rclpy.spin(aruco_generate)

    aruco_generate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()