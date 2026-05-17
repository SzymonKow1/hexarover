import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32  # Typ wiadomości do wysyłania kąta w świat ROS-a
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# Parametr kamery z Twojego skryptu
CAMERA_FOV_DEG = 60.0

def calculate_angle(x_center, image_width, fov):
    """Oblicza kąt odchylenia od centrum obrazu."""
    center_of_camera = image_width / 2.0
    pixel_difference = x_center - center_of_camera
    angle_per_pixel = fov / image_width
    return pixel_difference * angle_per_pixel

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info("Ładowanie modelu YOLOv8...")

        # 1. Inicjalizacja modelu YOLO
        self.model = YOLO("yolov8n.pt")
        
        # 2. Most OpenCV <-> ROS
        self.bridge = CvBridge()

        # 3. Subskrypcja obrazu (odbieramy klatki)
        self.subscription = self.create_subscription(
            Image,
            '/image_raw', 
            self.image_callback,
            10)
            
        # 4. NOWOŚĆ: Publikator kąta (wysyłamy wynik w świat)
        self.angle_pub = self.create_publisher(Float32, '/human_angle', 10)
        
        self.get_logger().info("Węzeł gotowy, czekam na obraz z /image_raw...")

    def image_callback(self, msg):
        try:
            # A. Konwersja wiadomości ROS na obraz OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Skalowanie z Twojego skryptu
            frame = cv2.resize(frame, (800, 600))
            image_width = frame.shape[1]

            # B. Inferencja YOLO (szukamy tylko klasy 0 - człowiek)
            results = self.model(frame, classes=[0], verbose=False)
            bounding_box = results[0].plot()

            # C. Obliczanie kątów dla wykrytych osób
            for r in results:
                boxes = r.boxes
                if len(boxes) > 0:
                    box = boxes[0]
                    x_center, y_center, width, height = box.xywh[0].tolist()

                    # Twoja matematyka
                    angle = calculate_angle(x_center, image_width, CAMERA_FOV_DEG)

                    # Wypisanie w profesjonalnym logu ROS 2
                    self.get_logger().info(f"Cel namierzony! Kąt: {angle:.1f} st.")

                    # Opublikowanie wyniku na temat /human_angle
                    angle_msg = Float32()
                    angle_msg.data = float(angle)
                    self.angle_pub.publish(angle_msg)

                    # Rysowanie na obrazku (do debugowania)
                    text = f"Angle: {angle:.1f} deg"
                    cv2.putText(bounding_box, text, (int(x_center) - 50, int(y_center) - 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # D. Podgląd na żywo
            cv2.imshow("Human Angle to Bobik", bounding_box)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Błąd przetwarzania klatki: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    
    # Sprzątanie przy wyłączaniu
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()