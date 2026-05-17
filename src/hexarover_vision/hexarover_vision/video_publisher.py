import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        
        # 1. Tworzymy publikatora, który wysyła obrazy na temat /image_raw
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        
        # 2. Most między OpenCV a ROS
        self.bridge = CvBridge()
        
        # tutaj modyfikuj ścieżke do filmiku testowego
        self.video_path = os.path.expanduser('~/ros2_ws/test_videos/test.mp4')
        self.cap = cv2.VideoCapture(self.video_path)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Nie moge otworzyc pliku: {self.video_path}")
            return

        # 4. Ustawiamy timer - funkcja odpali się ~30 razy na sekundę (jak 30 FPS)
        timer_period = 0.033 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Węzeł gotowy! Publikowanie wideo na /image_raw...')

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        # Jeśli film się skończył, zapętlamy go od nowa!
        if not ret:
            self.get_logger().info('Koniec filmu - zapętlam od początku.')
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()

        if ret:
            # Zamieniamy klatkę OpenCV na wiadomość ROS 2 i publikujemy
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    rclpy.spin(node)
    
    # Sprzątanie
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()