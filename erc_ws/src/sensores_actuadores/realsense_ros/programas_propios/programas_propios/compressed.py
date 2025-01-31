import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CompressedImagePublisher(Node):
    def __init__(self):
        super().__init__('realsense_compressed_image_publisher')
        
        # Publicar las imágenes comprimidas en el topic
        self.publisher = self.create_publisher(CompressedImage, '/camera/color/image_raw/compressed', 10)

        # Crear un temporizador para publicar imágenes cada 1 segundo
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Usamos OpenCV y CvBridge para capturar imágenes
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Usa la cámara RealSense si tienes configurada

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara.")
            return

        self.get_logger().info("Publicando imágenes comprimidas.")

    def timer_callback(self):
        # Captura una imagen de la cámara
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("No se pudo leer la imagen de la cámara.")
            return

        # Comprimir la imagen en formato JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # Calidad de JPEG (0-100)
        result, encoded_image = cv2.imencode('.jpg', frame, encode_param)

        if not result:
            self.get_logger().error("No se pudo comprimir la imagen.")
            return

        # Crear el mensaje de tipo CompressedImage
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_msg.header.frame_id = 'camera_color_optical_frame'  # O el frame_id adecuado
        compressed_msg.format = "jpeg"
        compressed_msg.data = np.array(encoded_image).tobytes()

        # Publicar el mensaje comprimido
        self.publisher.publish(compressed_msg)
        self.get_logger().info('Publicando una imagen comprimida.')

def main(args=None):
    rclpy.init(args=args)

    image_publisher = CompressedImagePublisher()

    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass

    image_publisher.cap.release()
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
