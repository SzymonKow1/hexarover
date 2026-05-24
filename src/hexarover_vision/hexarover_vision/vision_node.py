import rclpy
import math
import cv2
import numpy as np
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from ultralytics import YOLO

# ======================================================
# PARAMETRY KAMERY I LASERA
# ======================================================
CAMERA_FOV_DEG    = 77.0
LIDAR_OFFSET_DEG  = 0.0
FOV_LENGTH_M      = 3.0
FOV_ARC_STEPS     = 30
CLUSTER_DIST_M    = 0.3
SEARCH_WINDOW_DEG = 12.0
RAY_LENGTH_M      = 3.0
EMA_ALPHA         = 0.7

THROTTLE_SCAN_SEC = 0.1  


def calculate_angle(x_center, image_width, fov):
    return ((image_width / 2.0) - x_center) * (fov / image_width)


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()
        self.model  = YOLO("yolov8n.pt")

        # Subskrypcja z queue_size = 1 w celu uniknięcia opóźnień przetwarzania klatek
        self.scan_sub  = self.create_subscription(LaserScan, '/scan',      self.scan_callback,  1)
        self.image_sub = self.create_subscription(Image,     '/image_raw', self.image_callback, 1)

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

        self.latest_clusters = []
        self.latest_scan_stamp = None
        self.last_scan_time = self.get_clock().now()
        
        # Blokada wielowątkowa przetwarzania obrazu
        self.is_processing_frame = False

        # Deklaracja parametru debugowania w standardzie ROS 2
        self.declare_parameter('debug_mode', True)
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value

        self.create_timer(1.0, self.publish_fov_marker)
        self.get_logger().info("Węzeł Vision (Wektorowy Lidar + ROS Params) gotowy!")

    def scan_callback(self, msg):
        now = self.get_clock().now()
        if (now - self.last_scan_time).nanoseconds / 1e9 < THROTTLE_SCAN_SEC:
            return 
        self.last_scan_time = now

        # OPTYMALIZACJA: Wektorowa konwersja do NumPy zamiast wolnych pętli Python
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        valid_mask = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        if not np.any(valid_mask):
            self.latest_clusters = []
            return

        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        # Konwersja na współrzędne kartezjańskie
        xs = valid_ranges * np.cos(valid_angles)
        ys = valid_ranges * np.sin(valid_angles)

        # Tworzymy tablicę punktów [x, y, range, angle]
        pts = np.stack([xs, ys, valid_ranges, valid_angles], axis=1)

        # Szybkie klastrowanie przy użyciu różnic odległości euklidesowych sąsiadów
        dists = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
        split_indices = np.where(dists > CLUSTER_DIST_M)[0] + 1
        
        self.latest_clusters = np.split(pts, split_indices)
        self.latest_scan_stamp = msg.header.stamp

    def image_callback(self, msg):
        if self.is_processing_frame:
            return
        
        self.is_processing_frame = True
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.resize(frame, (320, 320))
            image_width = frame.shape[1]

            results = self.model.track(
                frame, classes=[0], max_det=1,
                verbose=False, conf=0.5, persist=True
            )
            
            if self.debug_mode:
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

                self.publish_yolo_ray(angle_laser_rad, msg.header.stamp)
                self.publish_search_window(angle_laser_rad, msg.header.stamp)

                if self.latest_clusters and self.latest_scan_stamp:
                    self.find_and_mark_target(self.latest_clusters, self.latest_scan_stamp)

                if self.debug_mode:
                    cv2.putText(display, f"Angle: {angle_deg:.1f} deg",
                                (int(x_center) - 50, int(y_center) - 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            else:
                self.yolo_angle_rad     = None
                self.yolo_angle_cam_deg = None
                self.clear_yolo_markers()

            if self.debug_mode:
                cv2.imshow("Hexarover AI Vision", display)
                cv2.waitKey(1)
            
        finally:
            self.is_processing_frame = False

    def find_and_mark_target(self, clusters, scan_stamp):
        half_window = math.radians(SEARCH_WINDOW_DEG)

        candidates = []
        for cluster in clusters:
            # cluster jest teraz wycinkiem tablicy NumPy [x, y, range, angle]
            angle_mean = np.mean(cluster[:, 3])
            diff = abs(math.atan2(
                math.sin(angle_mean - self.yolo_angle_rad),
                math.cos(angle_mean - self.yolo_angle_rad)
            ))
            if diff > half_window:
                continue
            dist_min = np.min(cluster[:, 2])
            candidates.append((dist_min, cluster))

        if not candidates:
            self.smooth_angle = EMA_ALPHA * self.yolo_angle_cam_deg + (1 - EMA_ALPHA) * self.smooth_angle
            
            angle_msg = Float32()
            angle_msg.data = float(self.smooth_angle)
            self.angle_pub.publish(angle_msg)

            dist_msg = Float32()
            dist_msg.data = float(self.smooth_dist) 
            self.dist_pub.publish(dist_msg)
            return  

        candidates.sort(key=lambda c: c[0])

        if len(candidates) >= 2:
            c1 = candidates[0][1]
            c2 = candidates[1][1]
            cx1 = np.mean(c1[:, 0])
            cy1 = np.mean(c1[:, 1])
            cx2 = np.mean(c2[:, 0])
            cy2 = np.mean(c2[:, 1])
            dist_between = math.hypot(cx2 - cx1, cy2 - cy1)

            if dist_between <= 0.6:
                top = candidates[:2]
            else:
                top = candidates[:1]
        else:
            top = candidates[:1]

        best_distance = min(c[0] for c in top)

        if self.smooth_dist == 0.0:
            self.smooth_angle = self.yolo_angle_cam_deg
            self.smooth_dist  = best_distance
        else:
            self.smooth_angle = EMA_ALPHA * self.yolo_angle_cam_deg + (1 - EMA_ALPHA) * self.smooth_angle
            self.smooth_dist  = EMA_ALPHA * best_distance           + (1 - EMA_ALPHA) * self.smooth_dist
            
        angle_msg      = Float32()
        angle_msg.data = float(self.smooth_angle)
        self.angle_pub.publish(angle_msg)

        dist_msg      = Float32()
        dist_msg.data = float(self.smooth_dist)
        self.dist_pub.publish(dist_msg)

        marker_x = self.smooth_dist * math.cos(self.yolo_angle_rad)
        marker_y = self.smooth_dist * math.sin(self.yolo_angle_rad)

        marker                  = Marker()
        marker.header.frame_id  = 'base_link'
        marker.header.stamp     = scan_stamp
        marker.ns               = 'human_target'
        marker.id               = 0
        marker.type             = Marker.SPHERE
        marker.action           = Marker.ADD
        marker.pose.position.x  = marker_x
        marker.pose.position.y  = marker_y
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

    def publish_yolo_ray(self, angle_rad, msg_stamp):
        marker                  = Marker()
        marker.header.frame_id  = 'base_link'
        marker.header.stamp     = msg_stamp
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

    def publish_search_window(self, angle_rad, msg_stamp):
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
        marker.header.frame_id  = 'base_link'
        marker.header.stamp     = msg_stamp
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
            marker.header.frame_id = 'base_link'
            marker.header.stamp    = self.get_clock().now().to_msg()
            marker.ns              = ns
            marker.id              = 0
            marker.action          = Marker.DELETE
            pub.publish(marker)

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
        marker.header.frame_id  = 'base_link'
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()