import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
import numpy as np

class EfficientPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('efficient_pointcloud_publisher')
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
        )
        self.publisher_ = self.create_publisher(PointCloud2, 'point_cloud2', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_pointcloud)  # Publica cada 0.1 segundos (10 Hz)

    def publish_pointcloud(self):
        points = np.random.rand(1000, 3).astype(np.float32)  # Simulación de datos; reemplaza con la cámara
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        # Empaquetar los datos usando `point_cloud2` de forma optimizada
        pc2_msg = point_cloud2.create_cloud(header, [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ], points)

        self.publisher_.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EfficientPointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
