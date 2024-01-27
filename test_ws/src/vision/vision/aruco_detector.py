import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
import matplotlib.pyplot as plt

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


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')
        
        self.publisher_marker = self.create_publisher(String, 'marker', 10) # Publisher for String message
        self.publisher_image = self.create_publisher(Image, 'marker_image', 10)  # Publisher for Image message

        self.timer_period = 0.4  # seconds
        self.timer = self.create_timer(self.timer_period, self.aruco_callback)
        self.bridge = CvBridge()

    def aruco_callback(self):
        
        # Load input image
        path_image = "/home/miguelub/Documents/GitHub/ERC2024/test_ws/src/vision/vision/test_images/singlemarkerssource.png"  # !!! Change the path to your image !!!
        image = cv2.imread(path_image) 

        # Convert the image to grayscale    
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Define the ArUco dictionary (you can choose different dictionaries)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

        # Define parameters for the marker detection
        parameters = cv2.aruco.DetectorParameters()

        # Detect markers in the image
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Draw the detected markers on the image
        image_with_markers = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)
  
        # Convert the image to ROS Image message
        ros_image_msg = self.bridge.cv2_to_imgmsg(image_with_markers, encoding="bgr8")

        # Publish the ROS Image message
        self.publisher_image.publish(ros_image_msg)
        self.get_logger().info("ArUco image published")

        # Check if ArUco markers are detected
        if len(corners) > 0:
            marker_string = String()
            marker_string.data = f"ArUco markers detected: {', '.join(map(str, ids.flatten()))}"
            # Publish the marker string
            self.publisher_marker.publish(marker_string)
            self.get_logger().info(marker_string.data)

def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArucoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()