import rclpy
import math
import cv2
from rclpy.node import Node

# Importy wiadomości ROS 2
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker

# Importy dla obrazu i AI
from cv_bridge import CvBridge
from ultralytics import YOLO

# Parametr kamery
CAMERA_FOV_DEG = 77.0

def calculate_angle(x_center, image_width, fov):
    """Oblicza kąt odchylenia od centrum obrazu w stopniach."""
    center_of_camera = image_width / 2.0
    # W ROS kąty w lewo są dodatnie, w prawo ujemne. 
    # Piksele rosną w prawo, więc odwracamy logikę:
    pixel_difference = center_of_camera - x_center 
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

        # 3. Subskrypcja obrazu (Kamera)
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
            
        # 4. Subskrypcja LiDARu (Laser)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.latest_scan = None
        
        # 5. Publikator kąta 
        self.angle_pub = self.create_publisher(Float32, '/human_angle', 10)

        # 6. Publikator Markera dla RViz2
        self.marker_pub = self.create_publisher(Marker, '/human_marker', 10)
        
        self.get_logger().info("Węzeł gotowy, fuzja sensoryczna uruchomiona!")

    def scan_callback(self, msg):
        """Aktualizuje najnowszy odczyt z lasera w tle."""
        self.latest_scan = msg

    def image_callback(self, msg):
        try:
            # A. Konwersja i skalowanie
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.resize(frame, (800, 600))
            image_width = frame.shape[1]

            # B. Inferencja YOLO
            results = self.model(frame, classes=[0], verbose=False) # class 0 = person
            bounding_box = results[0].plot()

            # C. Fuzja i obliczenia
            for r in results:
                boxes = r.boxes
                if len(boxes) > 0:
                    # Bierzemy pierwszego znalezionego człowieka
                    box = boxes[0]
                    x_center, y_center, width, height = box.xywh[0].tolist()

                    # Obliczenie kąta w stopniach i radianach
                    angle_deg = calculate_angle(x_center, image_width, CAMERA_FOV_DEG)
                    angle_rad = math.radians(angle_deg)

                    # Publikacja samego kąta
                    angle_msg = Float32()
                    angle_msg.data = float(angle_deg)
                    self.angle_pub.publish(angle_msg)

                    # ==========================================
                    # FUZJA SENSORYCZNA Z LIDAREM (Jeśli mamy dane)
                    # ==========================================
                    if self.latest_scan is not None:
                        scan = self.latest_scan
                        
                        # Szukamy, który promień lasera odpowiada naszemu kątowi
                        index_float = (angle_rad - scan.angle_min) / scan.angle_increment
                        index = int(round(index_float))

                        # Bierzemy kilka promieni wokół celu, żeby nie "przestrzelić"
                        window_size = 5 
                        valid_distances = []
                        
                        min_i = max(0, index - window_size)
                        max_i = min(len(scan.ranges), index + window_size + 1)
                        
                        for i in range(min_i, max_i):
                            d = scan.ranges[i]
                            # Ignorujemy błędy i nieskończoność
                            if not math.isinf(d) and scan.range_min < d < scan.range_max:
                                valid_distances.append(d)

                        if valid_distances:
                            # Wybieramy najbliższy punkt (to jest nasz człowiek)
                            distance = min(valid_distances)
                            
                            # Trygonometria - wyliczamy X, Y względem robota
                            target_x = distance * math.cos(angle_rad)
                            target_y = distance * math.sin(angle_rad)
                            
                            self.get_logger().info(f"CEL: Kąt {angle_deg:.1f}st | Dystans: {distance:.2f}m | Współrzędne: X:{target_x:.2f}, Y:{target_y:.2f}")
                            
                            # Rysujemy cel w RViz
                            self.publish_human_marker(target_x, target_y)

                            # Pokazujemy odległość na ekranie YOLO
                            text_dist = f"Dist: {distance:.2f}m"
                            cv2.putText(bounding_box, text_dist, (int(x_center) - 50, int(y_center) - 20),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

                    # Pokazujemy kąt na ekranie YOLO
                    text_angle = f"Angle: {angle_deg:.1f} deg"
                    cv2.putText(bounding_box, text_angle, (int(x_center) - 50, int(y_center) - 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # D. Podgląd na żywo
            cv2.imshow("Hexarover AI Vision", bounding_box)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Błąd przetwarzania klatki: {e}")

    def publish_human_marker(self, x, y):
        """Tworzy i wysyła zielony cylinder do RViz2 w miejscu wykrycia człowieka."""
        marker = Marker()
        marker.header.frame_id = 'lidar_link' # Znacznik osadzony względem lasera
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'rescue_target'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Pozycja wyliczona z fuzji danych
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.5 # Wyrównanie w pionie
        
        # Skala (Wielkość markera w metrach)
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 1.0
        
        # Kolor (Jasnozielony, przezroczysty)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8 
        
        # Marker znika automatycznie po 0.5 sekundy, jeśli cel zniknie z kamery
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 500000000 
        
        self.marker_pub.publish(marker)

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