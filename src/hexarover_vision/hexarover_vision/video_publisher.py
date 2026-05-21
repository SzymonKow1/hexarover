import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # 1. Tworzymy publikatora, który wysyła obrazy na temat /image_raw
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        
        # 2. Most między OpenCV a ROS
        self.bridge = CvBridge()
        
        # 3. Otwieramy domyślną kamerę systemową (indeks 0 to zazwyczaj /dev/video0 w Ubuntu)
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error("Nie mogę otworzyć domyślnej kamery (indeks 0). Sprawdź podłączenie!")
            return

        # 4. Ustawiamy timer - funkcja odpali się ~30 razy na sekundę (30 FPS)
        timer_period = 0.033 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Węzeł gotowy! Publikowanie obrazu z kamery na /image_raw...')

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # Zamieniamy klatkę OpenCV na wiadomość ROS 2 i publikujemy
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(msg)
        else:
            self.get_logger().warning('Nie udało się pobrać klatki z kamery. Czy kamera została odłączona?', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Sprzątanie
        node.get_logger().info('Zamykanie węzła...')
        if node.cap.isOpened():
            node.cap.release()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()