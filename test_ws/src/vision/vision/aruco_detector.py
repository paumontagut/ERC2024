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
        
        self.declare_parameter('aruco_dict', 'DICT_4X4_250')
        self.declare_parameter('camera_activation', False)

        self.publisher_marker = self.create_publisher(String, 'marker', 10) # Publisher for String message
        self.publisher_image = self.create_publisher(Image, 'marker_image', 10)  # Publisher for Image message

        self.timer_period = 0.2  # seconds
        self.timer = self.create_timer(self.timer_period, self.aruco_callback)

        self.bridge = CvBridge()
        self.image = None

        # Aruco Dictionary Parameter
        self.aruco_type = self.get_parameter('aruco_dict').get_parameter_value().string_value
        param_aruco_type = rclpy.parameter.Parameter('aruco_dict', rclpy.Parameter.Type.STRING, "DICT_4X4_250")
        # Camera Activation Parameter
        self.camera_activation = self.get_parameter('camera_activation').get_parameter_value().bool_value
        param_camera_activation = rclpy.parameter.Parameter('camera_activation', rclpy.Parameter.Type.BOOL, False)

        # Set parameters
        all_parameters = [param_aruco_type, param_camera_activation]
        self.set_parameters(all_parameters)

        if self.camera_activation:
            self.subcriber_camera = self.create_subscription(Image, 'color/image_raw', self.camera_callback, 10) # Subscriber for Camera Image

        else:    
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def camera_callback(self, msg):

        try:
            # Convert the ROS Image message to OpenCV Image
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        except Exception as e:
            self.get_logger().error(f"Error in camera_callback: {e}")

    def aruco_callback(self):

        if self.camera_activation and self.image is not None:
            image = self.image.copy()
        elif self.cap.isOpened():
            ret, frame = self.cap.read()
        
        self.aruco_show(frame)

    def pose_estimation(self, frame, aruco_dict, matrix_coefficients, distortion_coefficients):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dict])
        parameters = cv2.aruco.DetectorParameters()

        corners, ids, rejected = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters, 
                                                         cameraMatrix=matrix_coefficients, distCoeff=distortion_coefficients) 

        if len(corners) > 0:
            for i in range(0, len(ids)):
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients, distortion_coefficients)  

                cv2.aruco.drawDetectedMarkers(frame, corners)

                #cv2.aruco.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec[i], tvec[i], 0.01)

        return frame

    def aruco_show(self, image):
                # Convert the image to grayscale    
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Define the ArUco dictionary (you can choose different dictionaries)
        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[self.aruco_type])

        # Define parameters for the marker detection
        parameters = cv2.aruco.DetectorParameters()

        # Detect markers in the image
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Check if ArUco markers are detected
        if len(corners) > 0:

            # Draw the detected markers on the image
            image_with_markers = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)

            # Convert the image to ROS Image message
            ros_image_msg = self.bridge.cv2_to_imgmsg(image_with_markers, encoding="bgr8")
            
            # Publish the ROS Image message
            self.publisher_image.publish(ros_image_msg)
            self.get_logger().info("ArUco image published")
            
            marker_string = String()
            marker_string.data = f"ArUco markers detected: {', '.join(map(str, ids.flatten()))}"
            # Publish the marker string
            self.publisher_marker.publish(marker_string)
            self.get_logger().info(marker_string.data)

            for (markerCorner, markerID) in zip(corners, ids.flatten()):

                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # Draw the bounding box of the ArUco detection
                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

                # Compute and draw the center (x, y)-coordinates of the ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

        # Draw the ArUco marker ID on the image
        cv2.imshow("ArUco Marker Detection", image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArucoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()