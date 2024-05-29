# image_subscriber.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, 'video_frames', self.listener_callback, 10)
        self.br = CvBridge()

    def listener_callback(self, data):
        current_time = self.get_clock().now().to_msg()
        frame_time = data.header.stamp

        # Acessar os componentes de tempo usando methods
        current_sec = current_time.sec
        current_nanosec = current_time.nanosec
        frame_sec = frame_time.sec
        frame_nanosec = frame_time.nanosec

        latency = (current_sec - frame_sec) + (current_nanosec - frame_nanosec) / 1e9
        self.get_logger().info(f'Latency: {latency:.6f} seconds')

        frame = self.br.imgmsg_to_cv2(data)

        # Desenhar a latência na imagem
        text = f'Latency: {latency:.6f} seconds'
        font = cv2.FONT_HERSHEY_SIMPLEX
        org = (10, 30)
        font_scale = 1
        color = (0, 255, 0)
        thickness = 2
        frame = cv2.putText(frame, text, org, font, font_scale, color, thickness, cv2.LINE_AA)

        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
