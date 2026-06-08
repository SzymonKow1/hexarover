import os
import sys
import time
import rclpy
import math
import cv2
import numpy as np
from rclpy.node import Node

try:
    os.sched_setaffinity(0, {2, 3})
except AttributeError:
    pass

from sensor_msgs.msg import LaserScan, Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from ultralytics import YOLO

CAMERA_FOV_DEG    = 77.0
LIDAR_OFFSET_DEG  = -90.0
FOV_LENGTH_M      = 3.0
FOV_ARC_STEPS     = 30
CLUSTER_DIST_M    = 0.3
SEARCH_WINDOW_DEG = 12.0
RAY_LENGTH_M      = 3.0
EMA_ALPHA         = 0.7
THROTTLE_SCAN_SEC = 0.2

def calculate_angle(x_center, image_width, fov):
    return ((image_width / 2.0) - x_center) * (fov / image_width)

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()

        self.model = YOLO("/home/bobik/ros2_ws/yolov8n_256_ncnn_model")

        # Bezposrednie czytanie kamery
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
        if not self.cap.isOpened():
            self.get_logger().error("Nie mozna otworzyc kamery!")
            sys.exit()

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.image_pub = self.create_publisher(Image, '/image_raw', qos_profile_sensor_data)
        self.fov_pub = self.create_publisher(Marker, '/camera_fov', 10)
        self.cluster_pub = self.create_publisher(MarkerArray, '/lidar_clusters', 10)
        self.ray_pub = self.create_publisher(Marker, '/yolo_ray', 10)
        self.window_pub = self.create_publisher(Marker, '/yolo_window', 10)
        self.target_pub = self.create_publisher(Marker, '/human_marker', 10)
        self.angle_pub = self.create_publisher(Float32, '/human_angle', 10)
        self.dist_pub = self.create_publisher(Float32, '/human_distance', 10)

        self.lidar_offset_rad = math.radians(LIDAR_OFFSET_DEG)
        self.yolo_angle_rad = None
        self.yolo_angle_cam_deg = None
        self.smooth_angle = 0.0
        self.smooth_dist = 0.0
        self.latest_clusters = []
        self.latest_scan_stamp = None
        self.last_scan_time = self.get_clock().now()
        self.is_processing_frame = False
        self.yolo_ready = False

        self.create_timer(1.0, self.publish_fov_marker)
        self.create_timer(0.1, self.camera_yolo_callback)
        self.get_logger().info("Vision node gotowy!")

    def scan_callback(self, msg):
        now = self.get_clock().now()
        if (now - self.last_scan_time).nanoseconds / 1e9 < THROTTLE_SCAN_SEC:
            return
        self.last_scan_time = now

        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Maska poprawnych odczytow
        valid_mask = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)

        # Ograniczenie do FOV kamery + margines 10 stopni
        camera_offset = math.radians(LIDAR_OFFSET_DEG)
        half_fov = math.radians(CAMERA_FOV_DEG / 2.0 + 10.0)
        angle_diff = angles - camera_offset
        angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi
        angle_mask = np.abs(angle_diff) < half_fov
        valid_mask = valid_mask & angle_mask


        if not np.any(valid_mask):
            self.latest_clusters = []
            return

        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        xs = valid_ranges * np.cos(valid_angles)
        ys = valid_ranges * np.sin(valid_angles)
        pts = np.stack([xs, ys, valid_ranges, valid_angles], axis=1)
        dists = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
        split_indices = np.where(dists > CLUSTER_DIST_M)[0] + 1
        self.latest_clusters = np.split(pts, split_indices)
        self.latest_scan_stamp = msg.header.stamp
        self.publish_clusters(self.latest_clusters, msg.header.stamp)

    def camera_yolo_callback(self):
        if self.is_processing_frame:
            return
        self.is_processing_frame = True

        try:
            ret, frame = self.cap.read()
            if not ret:
                return

            image_width = frame.shape[1]
            results = self.model.track(
                frame, classes=[0], max_det=1,
                verbose=False, conf=0.5, persist=True,
                imgsz=256, iou=0.3
            )

            self.yolo_ready = True
            display = results[0].plot()

            x_center = None
            y_center = None
            for r in results:
                if len(r.boxes) == 0:
                    continue
                x_center, y_center, _, _ = r.boxes[0].xywh[0].tolist()

            if x_center is not None:
                angle_deg = calculate_angle(x_center, image_width, CAMERA_FOV_DEG)
                angle_cam_rad = math.radians(angle_deg)
                angle_laser_rad = math.atan2(
                    math.sin(angle_cam_rad + self.lidar_offset_rad),
                    math.cos(angle_cam_rad + self.lidar_offset_rad)
                )
                self.yolo_angle_rad = angle_laser_rad
                self.yolo_angle_cam_deg = angle_deg

                stamp = self.get_clock().now().to_msg()
                self.publish_yolo_ray(angle_laser_rad, stamp)
                self.publish_search_window(angle_laser_rad, stamp)

                if self.latest_clusters and self.latest_scan_stamp:
                    self.find_and_mark_target(self.latest_clusters, self.latest_scan_stamp)
            else:
                self.yolo_angle_rad = None
                self.yolo_angle_cam_deg = None
                self.clear_yolo_markers()

            try:
                msg = self.bridge.cv2_to_imgmsg(display, "bgr8")
                msg.header.frame_id = "laser"
                msg.header.stamp = self.get_clock().now().to_msg()
                self.image_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Blad publikacji obrazu: {e}")

        finally:
            self.is_processing_frame = False

    def find_and_mark_target(self, clusters, scan_stamp):
        half_window = math.radians(SEARCH_WINDOW_DEG)
        candidates = []
        for cluster in clusters:
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
            if self.yolo_angle_cam_deg is not None:
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
            cx1, cy1 = np.mean(c1[:, 0]), np.mean(c1[:, 1])
            cx2, cy2 = np.mean(c2[:, 0]), np.mean(c2[:, 1])
            if math.hypot(cx2 - cx1, cy2 - cy1) <= 0.6:
                top = candidates[:2]
            else:
                top = candidates[:1]
        else:
            top = candidates[:1]

        best_distance = min(c[0] for c in top)

        if self.smooth_dist == 0.0:
            self.smooth_angle = self.yolo_angle_cam_deg
            self.smooth_dist = best_distance
        else:
            self.smooth_angle = EMA_ALPHA * self.yolo_angle_cam_deg + (1 - EMA_ALPHA) * self.smooth_angle
            self.smooth_dist = EMA_ALPHA * best_distance + (1 - EMA_ALPHA) * self.smooth_dist

        angle_msg = Float32()
        angle_msg.data = float(self.smooth_angle)
        self.angle_pub.publish(angle_msg)
        dist_msg = Float32()
        dist_msg.data = float(self.smooth_dist)
        self.dist_pub.publish(dist_msg)

        self.get_logger().info(f"CZLOWIEK | Dist: {self.smooth_dist:.2f}m | Kat: {self.smooth_angle:.1f}deg", throttle_duration_sec=0.5)

        marker_x = self.smooth_dist * math.cos(self.yolo_angle_rad)
        marker_y = self.smooth_dist * math.sin(self.yolo_angle_rad)

        marker = Marker()
        marker.header.frame_id = 'laser'
        marker.header.stamp = scan_stamp
        marker.ns = 'human_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = marker_x
        marker.pose.position.y = marker_y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 1
        marker.lifetime.nanosec = 0
        self.target_pub.publish(marker)

    def publish_yolo_ray(self, angle_rad, msg_stamp):
        marker = Marker()
        marker.header.frame_id = 'laser'
        marker.header.stamp = msg_stamp
        marker.ns = 'yolo_ray'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 1
        marker.lifetime.nanosec = 0
        marker.points.append(Point(x=0.0, y=0.0, z=0.0))
        marker.points.append(Point(
            x=RAY_LENGTH_M * math.cos(angle_rad),
            y=RAY_LENGTH_M * math.sin(angle_rad),
            z=0.0
        ))
        self.ray_pub.publish(marker)

    def publish_search_window(self, angle_rad, msg_stamp):
        half_w = math.radians(SEARCH_WINDOW_DEG)
        pts = [Point(x=0.0, y=0.0, z=0.0)]
        pts.append(Point(
            x=RAY_LENGTH_M * math.cos(angle_rad + half_w),
            y=RAY_LENGTH_M * math.sin(angle_rad + half_w),
            z=0.0
        ))
        for i in range(FOV_ARC_STEPS + 1):
            a = angle_rad + half_w - i * (2 * half_w / FOV_ARC_STEPS)
            pts.append(Point(x=RAY_LENGTH_M * math.cos(a), y=RAY_LENGTH_M * math.sin(a), z=0.0))
        pts.append(Point(x=0.0, y=0.0, z=0.0))

        marker = Marker()
        marker.header.frame_id = 'laser'
        marker.header.stamp = msg_stamp
        marker.ns = 'yolo_window'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.lifetime.sec = 1
        marker.lifetime.nanosec = 0
        marker.points = pts
        self.window_pub.publish(marker)

    def clear_yolo_markers(self):
        for pub, ns in [(self.ray_pub, 'yolo_ray'), (self.window_pub, 'yolo_window')]:
            marker = Marker()
            marker.header.frame_id = 'laser'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ns
            marker.id = 0
            marker.action = Marker.DELETE
            pub.publish(marker)

    def publish_clusters(self, clusters, scan_stamp):
        marker_array = MarkerArray()
        for idx, cluster in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = 'laser'
            marker.header.stamp = scan_stamp
            marker.ns = 'clusters'
            marker.id = idx
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = 0.08
            marker.scale.y = 0.08
            marker.color.r = 1.0
            marker.color.g = 0.4
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime.sec = 1
            marker.lifetime.nanosec = 0
            for p in cluster:
                marker.points.append(Point(x=float(p[0]), y=float(p[1]), z=0.0))
            marker_array.markers.append(marker)
        self.cluster_pub.publish(marker_array)

    def publish_fov_marker(self):
        half_fov = math.radians(CAMERA_FOV_DEG / 2.0)
        offset = self.lidar_offset_rad
        pts = [Point(x=0.0, y=0.0, z=0.0)]
        pts.append(Point(
            x=FOV_LENGTH_M * math.cos(offset + half_fov),
            y=FOV_LENGTH_M * math.sin(offset + half_fov),
            z=0.0
        ))
        for i in range(FOV_ARC_STEPS + 1):
            a = offset + half_fov - i * (2 * half_fov / FOV_ARC_STEPS)
            pts.append(Point(x=FOV_LENGTH_M * math.cos(a), y=FOV_LENGTH_M * math.sin(a), z=0.0))
        pts.append(Point(x=0.0, y=0.0, z=0.0))

        marker = Marker()
        marker.header.frame_id = 'laser'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'camera_fov'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = 0.2
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.color.a = 0.7
        marker.lifetime.sec = 2
        marker.lifetime.nanosec = 0
        marker.points = pts
        self.fov_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'cap') and node.cap.isOpened():
            node.cap.release()
        node.destroy_node()
        rclpy.try_shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
