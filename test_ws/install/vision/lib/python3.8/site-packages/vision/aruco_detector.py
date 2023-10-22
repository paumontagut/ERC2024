import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fiducial_msgs.msg import FiducialArray


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')
        self.subcriber = self.create_subscription(FiducialArray, '/fiducial_vertices', self.fiducials_callback, 10)
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer_period = 0.4  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def fiducials_callback(self, fiducials):
        self.vertices = fiducials.fiducials[0]
    
    def timer_callback(self):
        msg = String()
        if len(self.vertices) != 0:
            msg.data = 'ArUco detected'
        else:
            msg.data = 'ArUco not detected'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArucoDetector()

    rclpy.spin(aruco_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()