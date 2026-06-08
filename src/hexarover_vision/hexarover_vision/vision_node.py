# --- START OF FILE vision_node.py ---

import os
import sys
import time

# TWARDE OGRANICZENIE RDZENI DLA DETEKCJI (YOLO / NCNN):
# Blokujemy proces tylko na rdzeniach 2 i 3.
# Rdzenie 0 i 1 pozostają wolne dla komunikacji ROS 2 i systemu operacyjnego.
try:
    os.sched_setaffinity(0, {2, 3})
except AttributeError:
    pass

import rclpy
import math
import cv2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from ultralytics import YOLO

CAMERA_FOV_DEG    = 77.0
LIDAR_OFFSET_DEG  = -90.0
FOV_LENGTH_M      = 3.0
FOV_ARC_STEPS     = 30
CLUSTER_DIST_M    = 0.3
SEARCH_WINDOW_DEG = 12.0
RAY_LENGTH_M      = 3.0
EMA_ALPHA         = 0.7


def calculate_angle(x_center, image_width, fov):
    return ((image_width / 2.0) - x_center) * (fov / image_width)


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()
        
        # 1. Ładowanie modelu NCNN
        self.model = YOLO("yolov8n_256_ncnn_model")
        # 2. Bezpośrednie otwarcie kamery (unikanie opóźnień sieciowych ROS 2)
        # Indeks 1 (zmień na 0, jeśli kamera nie ruszy)
        self.camera_index = 0
        self.cap = cv2.VideoCapture(self.camera_index)
        
        # Optymalizacja rozdzielczości
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error(f"Nie można otworzyć kamery o indeksie {self.camera_index}!")
            sys.exit()

        # 3. Subskrypcja LiDAR-u
        self.scan_sub  = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # 4. Publikatory (w tym opcjonalna publikacja obrazu do RViz z profilem qos_profile_sensor_data)
        self.image_pub   = self.create_publisher(Image,       '/image_raw',       qos_profile_sensor_data)
        self.fov_pub     = self.create_publisher(Marker,      '/camera_fov',      10)
        self.cluster_pub = self.create_publisher(MarkerArray, '/lidar_clusters',   10)
        self.ray_pub     = self.create_publisher(Marker,      '/yolo_ray',         10)
        self.window_pub  = self.create_publisher(Marker,      '/yolo_window',      10)
        self.target_pub  = self.create_publisher(Marker,      '/human_marker',     10)
        self.angle_pub   = self.create_publisher(Float32,     '/human_angle',      10)
        self.dist_pub    = self.create_publisher(Float32,     '/human_distance',   10)

        self.lidar_offset_rad   = math.radians(LIDAR_OFFSET_DEG)
        self.yolo_angle_rad     = None
        self.yolo_angle_cam_deg = None

        self.smooth_angle = 0.0
        self.smooth_dist  = 0.0

        # Zmienna do pomiaru FPS na żywo
        self.last_frame_time = time.time()

        # 5. Timery
        self.create_timer(1.0, self.publish_fov_marker)
        
        # Pętla akwizycji i detekcji kamery: 0.067 s = 15 FPS
        self.create_timer(0.067, self.camera_yolo_callback)

        self.get_logger().info("Zunifikowany węzeł wizyjny NCNN gotowy i zoptymalizowany!")

    # ------------------------------------------------------------------ #

    def camera_yolo_callback(self):
        # Pomiar rzeczywistego czasu pętli do wyświetlenia FPS
        current_time = time.time()
        dt = current_time - self.last_frame_time
        self.last_frame_time = current_time
        fps = 1.0 / dt if dt > 0 else 0.0

        # Odczyt bezpośrednio z kamery (brak opóźnień ROS 2)
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Błąd odczytu klatki z kamery.', throttle_duration_sec=2.0)
            return

        #frame = cv2.resize(frame, (800, 600))
        image_width = frame.shape[1]

        # Detekcja YOLOv8 NCNN na świeżej lokalnej klatce
        results = self.model.track(
            frame, classes=[0], max_det=1,
            verbose=False, conf=0.5, persist=True,
            imgsz=256, iou=0.3
        )
        display = results[0].plot()

        x_center = None
        y_center = None

        for r in results:
            if len(r.boxes) == 0:
                continue
            x_center, y_center, _, _ = r.boxes[0].xywh[0].tolist()

        if x_center is not None:
            angle_deg = calculate_angle(x_center, image_width, CAMERA_FOV_DEG)

            angle_cam_rad   = math.radians(angle_deg)
            angle_laser_rad = math.atan2(
                math.sin(angle_cam_rad + self.lidar_offset_rad),
                math.cos(angle_cam_rad + self.lidar_offset_rad)
            )
            self.yolo_angle_rad     = angle_laser_rad
            self.yolo_angle_cam_deg = angle_deg

            self.publish_yolo_ray(angle_laser_rad)
            self.publish_search_window(angle_laser_rad)

            #cv2.putText(display, f"Angle: {angle_deg:.1f} deg",
                        #(int(x_center) - 50, int(y_center) - 50),
                        #cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            self.yolo_angle_rad     = None
            self.yolo_angle_cam_deg = None
            self.clear_yolo_markers()

        # Dodanie FPS do okna graficznego (kolor czerwony)
        #cv2.putText(display, f"FPS: {fps:.1f}", (20, 50), 
                    #cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        # Wyświetlenie okna bezpośrednio na pulpicie RPi 5
        #cv2.imshow("Hexarover AI Vision", display)
        #cv2.waitKey(1)

        # Publikowanie opisanego obrazu do ROS 2, aby RViz2 nadal mógł go wyświetlać
        try:
            msg = self.bridge.cv2_to_imgmsg(display, "bgr8")
            msg.header.frame_id = "laser"
            msg.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Błąd publikacji obrazu do ROS 2: {str(e)}")

    # ------------------------------------------------------------------ #

    def scan_callback(self, msg):
        points = []
        for i, d in enumerate(msg.ranges):
            if math.isinf(d) or math.isnan(d):
                continue
            if not (msg.range_min < d < msg.range_max):
                continue
            a = msg.angle_min + i * msg.angle_increment
            points.append((d * math.cos(a), d * math.sin(a), d, a))

        if not points:
            return

        clusters = []
        current  = [points[0]]
        for prev, curr in zip(points, points[1:]):
            if math.hypot(curr[0] - prev[0], curr[1] - prev[1]) > CLUSTER_DIST_M:
                clusters.append(current)
                current = [curr]
            else:
                current.append(curr)
        clusters.append(current)

        self.publish_clusters(clusters)

        if self.yolo_angle_rad is not None:
            self.find_and_mark_target(clusters)

    # ------------------------------------------------------------------ #

    def find_and_mark_target(self, clusters):
        half_window = math.radians(SEARCH_WINDOW_DEG)

        candidates = []
        for cluster in clusters:
            angle_mean = sum(p[3] for p in cluster) / len(cluster)
            diff = abs(math.atan2(
                math.sin(angle_mean - self.yolo_angle_rad),
                math.cos(angle_mean - self.yolo_angle_rad)
            ))
            if diff > half_window:
                continue
            dist_min = min(p[2] for p in cluster)
            candidates.append((dist_min, cluster))

        if not candidates:
            return

        candidates.sort(key=lambda c: c[0])

        if len(candidates) >= 2:
            c1 = candidates[0][1]
            c2 = candidates[1][1]
            cx1 = sum(p[0] for p in c1) / len(c1)
            cy1 = sum(p[1] for p in c1) / len(c1)
            cx2 = sum(p[0] for p in c2) / len(c2)
            cy2 = sum(p[1] for p in c2) / len(c2)
            dist_between = math.hypot(cx2 - cx1, cy2 - cy1)

            if dist_between <= 0.6:
                top = candidates[:2]
            else:
                top = candidates[:1]
        else:
            top = candidates[:1]

        all_points    = [p for _, cluster in top for p in cluster]
        cx            = sum(p[0] for p in all_points) / len(all_points)
        cy            = sum(p[1] for p in all_points) / len(all_points)
        best_distance = min(c[0] for c in top)

        self.smooth_angle = EMA_ALPHA * self.yolo_angle_cam_deg + (1 - EMA_ALPHA) * self.smooth_angle
        self.smooth_dist  = EMA_ALPHA * best_distance           + (1 - EMA_ALPHA) * self.smooth_dist

        self.get_logger().info(
            f"CZŁOWIEK | Dist: {self.smooth_dist:.2f}m | X:{cx:.2f} Y:{cy:.2f}")

        angle_msg      = Float32()
        angle_msg.data = float(self.smooth_angle)
        self.angle_pub.publish(angle_msg)

        dist_msg      = Float32()
        dist_msg.data = float(self.smooth_dist)
        self.dist_pub.publish(dist_msg)

        marker                  = Marker()
        marker.header.frame_id  = 'laser'
        marker.header.stamp     = self.get_clock().now().to_msg()
        marker.ns               = 'human_target'
        marker.id               = 0
        marker.type             = Marker.SPHERE
        marker.action           = Marker.ADD
        marker.pose.position.x  = cx
        marker.pose.position.y  = cy
        marker.pose.position.z  = 0.0
        marker.scale.x          = 0.3
        marker.scale.y          = 0.3
        marker.scale.z          = 0.3
        marker.color.r          = 1.0
        marker.color.g          = 0.0
        marker.color.b          = 0.0
        marker.color.a          = 1.0
        marker.lifetime.sec     = 0
        marker.lifetime.nanosec = 300_000_000
        self.target_pub.publish(marker)

    # ------------------------------------------------------------------ #

    def publish_yolo_ray(self, angle_rad):
        marker                  = Marker()
        marker.header.frame_id  = 'laser'
        marker.header.stamp     = self.get_clock().now().to_msg()
        marker.ns               = 'yolo_ray'
        marker.id               = 0
        marker.type             = Marker.LINE_LIST
        marker.action           = Marker.ADD
        marker.scale.x          = 0.03
        marker.color.r          = 1.0
        marker.color.g          = 1.0
        marker.color.b          = 0.0
        marker.color.a          = 1.0
        marker.lifetime.sec     = 0
        marker.lifetime.nanosec = 300_000_000
        marker.points.append(Point(x=0.0, y=0.0, z=0.0))
        marker.points.append(Point(
            x=RAY_LENGTH_M * math.cos(angle_rad),
            y=RAY_LENGTH_M * math.sin(angle_rad),
            z=0.0
        ))
        self.ray_pub.publish(marker)

    def publish_search_window(self, angle_rad):
        half_w = math.radians(SEARCH_WINDOW_DEG)
        origin = Point(x=0.0, y=0.0, z=0.0)
        pts    = []
        pts.append(origin)
        pts.append(Point(
            x=RAY_LENGTH_M * math.cos(angle_rad + half_w),
            y=RAY_LENGTH_M * math.sin(angle_rad + half_w),
            z=0.0
        ))
        for i in range(FOV_ARC_STEPS + 1):
            a = angle_rad + half_w - i * (2 * half_w / FOV_ARC_STEPS)
            pts.append(Point(x=RAY_LENGTH_M * math.cos(a), y=RAY_LENGTH_M * math.sin(a), z=0.0))
        pts.append(origin)

        marker                  = Marker()
        marker.header.frame_id  = 'laser'
        marker.header.stamp     = self.get_clock().now().to_msg()
        marker.ns               = 'yolo_window'
        marker.id               = 0
        marker.type             = Marker.LINE_STRIP
        marker.action           = Marker.ADD
        marker.scale.x          = 0.02
        marker.color.r          = 1.0
        marker.color.g          = 0.5
        marker.color.b          = 0.0
        marker.color.a          = 0.5
        marker.lifetime.sec     = 0
        marker.lifetime.nanosec = 300_000_000
        marker.points           = pts
        self.window_pub.publish(marker)

    def clear_yolo_markers(self):
        for pub, ns in [(self.ray_pub, 'yolo_ray'), (self.window_pub, 'yolo_window')]:
            marker                 = Marker()
            marker.header.frame_id = 'laser'
            marker.header.stamp    = self.get_clock().now().to_msg()
            marker.ns              = ns
            marker.id              = 0
            marker.action          = Marker.DELETE
            pub.publish(marker)

    def publish_clusters(self, clusters):
        marker_array = MarkerArray()
        for idx, cluster in enumerate(clusters):
            marker                  = Marker()
            marker.header.frame_id  = 'laser'
            marker.header.stamp     = self.get_clock().now().to_msg()
            marker.ns               = 'clusters'
            marker.id               = idx
            marker.type             = Marker.POINTS
            marker.action           = Marker.ADD
            marker.scale.x          = 0.08
            marker.scale.y          = 0.08
            marker.color.r          = 1.0
            marker.color.g          = 0.4
            marker.color.b          = 0.0
            marker.color.a          = 1.0
            marker.lifetime.sec     = 0
            marker.lifetime.nanosec = 200_000_000
            for p in cluster:
                marker.points.append(Point(x=p[0], y=p[1], z=0.0))
            marker_array.markers.append(marker)
        self.cluster_pub.publish(marker_array)

    def publish_fov_marker(self):
        half_fov = math.radians(CAMERA_FOV_DEG / 2.0)
        offset   = self.lidar_offset_rad
        origin   = Point(x=0.0, y=0.0, z=0.0)
        pts      = []
        pts.append(origin)
        pts.append(Point(
            x=FOV_LENGTH_M * math.cos(offset + half_fov),
            y=FOV_LENGTH_M * math.sin(offset + half_fov),
            z=0.0
        ))
        for i in range(FOV_ARC_STEPS + 1):
            a = offset + half_fov - i * (2 * half_fov / FOV_ARC_STEPS)
            pts.append(Point(x=FOV_LENGTH_M * math.cos(a), y=FOV_LENGTH_M * math.sin(a), z=0.0))
        pts.append(origin)

        marker                  = Marker()
        marker.header.frame_id  = 'laser'
        marker.header.stamp     = self.get_clock().now().to_msg()
        marker.ns               = 'camera_fov'
        marker.id               = 0
        marker.type             = Marker.LINE_STRIP
        marker.action           = Marker.ADD
        marker.scale.x          = 0.02
        marker.color.r          = 0.2
        marker.color.g          = 0.6
        marker.color.b          = 1.0
        marker.color.a          = 0.7
        marker.lifetime.sec     = 2
        marker.lifetime.nanosec = 0
        marker.points           = pts
        self.fov_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Zamykanie węzła wizyjnego i zwalnianie kamery...')
        # Bardzo ważne: zwalniamy zasób kamery przy wyjściu z programu
        if hasattr(node, 'cap') and node.cap.isOpened():
            node.cap.release()
        node.destroy_node()
        rclpy.try_shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()