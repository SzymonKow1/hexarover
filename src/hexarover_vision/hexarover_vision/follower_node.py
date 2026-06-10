"""
follower_node.py (Wersja Hybrydowa: RPi 5 + RPi 4)
-------------------------------------------------
Sterownik podążający za człowiekiem łączący PID z Nav2.
Wymaga działania Nav2 (na RPi 4) i węzła wizyjnego (na RPi 5).
"""
#pobranie twist_mux
# sudo apt update
# sudo apt install ros-jazzy-twist-mux
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
from collections import deque

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

# ======================================================
# PARAMETRY BEZPIECZEŃSTWA
# ======================================================
OBSTACLE_AVOID_DIST = 0.8       
CONE_ANGLE_DEG      = 20.0       
CMD_VEL_TIMEOUT     = 0.8       # Timeout z fizycznego robota
SEARCH_ANGULAR_VEL  = 0.35      # Maksymalna prędkość obrotu przy szukaniu

# ======================================================
# PARAMETRY PID I JAZDY (Z fizycznego robota)
# ======================================================
DESIRED_DISTANCE_M  = 0.75      # Dystans postoju
DIST_DEADZONE_M     = 0.08      
ANGLE_DEADZONE_DEG  = 8.0       

MAX_LINEAR_VEL  = 0.35           
MAX_ANGULAR_VEL = 0.40
MIN_LINEAR_VEL  = 0.15          # Minimalna prędkość dla silników Dagu
MIN_ANGULAR_VEL = 0.10          

# Strojenie PID (z fizycznego robota)
KP_ANG = 0.004   
KI_ANG = 0.0
KD_ANG = 0.02

KP_LIN = 0.6     
KI_LIN = 0.0
KD_LIN = 0.06

INVERT_STEERING = False  


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')

        # Klient akcji dla Nav2 (Nav2 działa na RPi 4, ale komunikuje się po DDS bez problemu)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.sub_dist  = self.create_subscription(Float32, '/human_distance', self.dist_callback, 10)
        self.sub_angle = self.create_subscription(Float32, '/human_angle', self.angle_callback, 10)
        self.sub_scan  = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_odom  = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publikujemy na specjalny temat dla twist_mux, nie bezpośrednio na silniki!
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_follower', 10)

        self.angle_history = deque(maxlen=10)
        self.last_trend_sign = 1.0  

        self.latest_dist = None
        self.latest_angle_deg = None
        self.last_human_update_time = self.get_clock().now()
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Ostatnia znana pozycja człowieka w globalnym układzie odometrii
        self.last_known_odom_x = 0.0
        self.last_known_odom_y = 0.0
        self.last_known_odom_yaw = 0.0

        self.obstacle_ahead = False
        self.state = 'IDLE'
        self.current_goal_handle = None
        self.nav2_failed_time = None

        self.integral_ang   = 0.0
        self.integral_lin   = 0.0
        self.prev_error_ang = 0.0
        self.prev_error_lin = 0.0
        self.last_pid_time  = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.decision_loop) # Przyspieszone do 20Hz jak w starym kodzie
        self.get_logger().info("Follower V20 (Hybryda: PID + Nav2 Action) gotowy na RPi 5!")

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def dist_callback(self, msg):
        # Filtr z poprzedniego kodu
        if msg.data < 0.15:
            return
        self.latest_dist = msg.data
        self.last_human_update_time = self.get_clock().now()

    def angle_callback(self, msg):
        self.latest_angle_deg = msg.data
        self.last_human_update_time = self.get_clock().now()
        
        self.angle_history.append(msg.data)
        self.update_movement_trend()

    def update_movement_trend(self):
        if len(self.angle_history) >= 3:
            start_angle = self.angle_history[0]
            end_angle = self.angle_history[-1]
            trend = end_angle - start_angle
            if abs(trend) > 2.0:
                self.last_trend_sign = 1.0 if trend > 0 else -1.0

    def scan_callback(self, msg):
        now = self.get_clock().now()
        time_since = (now - self.last_human_update_time).nanoseconds / 1e9
        human_recently_seen = (time_since <= 5.0 and self.latest_dist is not None)
        
        hx_base, hy_base = 0.0, 0.0
        if human_recently_seen:
            dx = self.last_known_odom_x - self.robot_x
            dy = self.last_known_odom_y - self.robot_y
            hx_base = dx * math.cos(-self.robot_yaw) - dy * math.sin(-self.robot_yaw)
            hy_base = dx * math.sin(-self.robot_yaw) + dy * math.cos(-self.robot_yaw)

        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        angles = (angles + np.pi) % (2 * np.pi) - np.pi 
        
        valid_mask = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        cone_rad = math.radians(CONE_ANGLE_DEG)
        cone_mask = np.abs(angles) < cone_rad
        roi_mask = valid_mask & cone_mask
        
        roi_ranges = ranges[roi_mask]
        roi_angles = angles[roi_mask]
        
        if len(roi_ranges) == 0:
            self.obstacle_ahead = False
            return

        if human_recently_seen:
            px = roi_ranges * np.cos(roi_angles)
            py = roi_ranges * np.sin(roi_angles)
            dist_to_human = np.hypot(px - hx_base, py - hy_base)
            obstacle_mask = dist_to_human >= 0.7
            obstacle_ranges = roi_ranges[obstacle_mask]
        else:
            obstacle_ranges = roi_ranges

        if len(obstacle_ranges) > 0:
            min_dist = np.min(obstacle_ranges)
        else:
            min_dist = float('inf')
        
        self.obstacle_ahead = min_dist < OBSTACLE_AVOID_DIST

    def reset_pid(self):
        self.integral_ang = 0.0
        self.integral_lin = 0.0
        self.prev_error_ang = 0.0
        self.prev_error_lin = 0.0

    def decision_loop(self):
        if self.latest_dist is None or self.latest_angle_deg is None:
            return

        now = self.get_clock().now()
        time_since = (now - self.last_human_update_time).nanoseconds / 1e9

        dt = (now - self.last_pid_time).nanoseconds / 1e9
        self.last_pid_time = now
        if dt <= 0.001:
            return

        if self.state == 'NAV2_AVOID':
            return
        
        if self.nav2_failed_time is not None:
            elapsed = (now - self.nav2_failed_time).nanoseconds / 1e9
            if elapsed < 0.5:
                self.send_zero_vel()
                return
            self.nav2_failed_time = None

        dist_to_human_pos = math.hypot(self.last_known_odom_x - self.robot_x, self.last_known_odom_y - self.robot_y)
        arrived_at_last_pos = (dist_to_human_pos <= 0.6)

        if time_since > CMD_VEL_TIMEOUT and arrived_at_last_pos:
            if self.state != 'SMART_SEARCH':
                self.get_logger().info("Dotarłem w miejsce zgubienia. Szukam kręcąc się...")
                self.state = 'SMART_SEARCH'
                self.reset_pid()

            spin_velocity = math.copysign(SEARCH_ANGULAR_VEL, self.last_trend_sign)
            if INVERT_STEERING:
                spin_velocity = -spin_velocity
                
            self.send_cmd_vel(0.0, spin_velocity)
            return

        if self.obstacle_ahead:
            self.get_logger().warn("Przeszkoda! Nav2 omija do EXACT pozycji człowieka.")
            self.send_zero_vel()
            self.reset_pid()
            
            self.send_nav_goal(self.last_known_odom_x, self.last_known_odom_y, self.last_known_odom_yaw)
            self.state = 'NAV2_AVOID'
            return

        if time_since <= CMD_VEL_TIMEOUT:
            if self.state != 'DIRECT_FOLLOW':
                self.get_logger().info("Widzę człowieka! Śledzę (PID).")
                self.cancel_current_goal()
                self.reset_pid()
                self.state = 'DIRECT_FOLLOW'
            
            dist = self.latest_dist
            angle_deg = self.latest_angle_deg
            angle_rad = math.radians(angle_deg)
            
            hx_base = dist * math.cos(angle_rad)
            hy_base = dist * math.sin(angle_rad)
            
            self.last_known_odom_x = self.robot_x + hx_base * math.cos(self.robot_yaw) - hy_base * math.sin(self.robot_yaw)
            self.last_known_odom_y = self.robot_y + hx_base * math.sin(self.robot_yaw) + hy_base * math.cos(self.robot_yaw)
            self.last_known_odom_yaw = self.robot_yaw + angle_rad

            error_ang = angle_deg
            error_lin = dist - DESIRED_DISTANCE_M

            self.integral_ang += error_ang * dt
            deriv_ang = (error_ang - self.prev_error_ang) / dt
            self.prev_error_ang = error_ang
            angular_z = (KP_ANG * error_ang + KI_ANG * self.integral_ang + KD_ANG * deriv_ang)

            self.integral_lin += error_lin * dt
            deriv_lin = (error_lin - self.prev_error_lin) / dt
            self.prev_error_lin = error_lin
            linear_x = (KP_LIN * error_lin + KI_LIN * self.integral_lin + KD_LIN * deriv_lin)

            # Aplikacja stref martwych i minimalnych progów mocy
            if abs(error_ang) < ANGLE_DEADZONE_DEG:
                angular_z = 0.0
                self.integral_ang = 0.0
            elif 0.0 < abs(angular_z) < MIN_ANGULAR_VEL:
                angular_z = 0.0 # Eliminacja buczenia silników bez ruchu

            if abs(error_lin) < DIST_DEADZONE_M:
                linear_x = 0.0
                self.integral_lin = 0.0
            elif 0.0 < abs(linear_x) < MIN_LINEAR_VEL:
                linear_x = 0.0 # Eliminacja buczenia silników bez ruchu

            if INVERT_STEERING:
                angular_z = -angular_z

            linear_x  = max(-MAX_LINEAR_VEL,  min(MAX_LINEAR_VEL,  linear_x))
            angular_z = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular_z))

            self.send_cmd_vel(linear_x, angular_z)

        else:
            if self.state != 'GOTO_LAST_POS':
                self.get_logger().info("Zgubiłem cel. Jadę w dokładne miejsce zniknięcia!")
                self.state = 'GOTO_LAST_POS'
                self.reset_pid()
            
            angle_to_target_odom = math.atan2(self.last_known_odom_y - self.robot_y, self.last_known_odom_x - self.robot_x)
            error_ang_rad = angle_to_target_odom - self.robot_yaw
            error_ang_rad = (error_ang_rad + math.pi) % (2 * math.pi) - math.pi
            
            error_ang = math.degrees(error_ang_rad) 
            error_lin = dist_to_human_pos - 0.2 

            self.integral_ang += error_ang * dt
            deriv_ang = (error_ang - self.prev_error_ang) / dt
            self.prev_error_ang = error_ang
            angular_z = (KP_ANG * error_ang + KI_ANG * self.integral_ang + KD_ANG * deriv_ang)

            self.integral_lin += error_lin * dt
            deriv_lin = (error_lin - self.prev_error_lin) / dt
            self.prev_error_lin = error_lin
            linear_x = (KP_LIN * error_lin + KI_LIN * self.integral_lin + KD_LIN * deriv_lin)

            if abs(error_ang) < ANGLE_DEADZONE_DEG:
                angular_z = 0.0
            elif 0.0 < abs(angular_z) < MIN_ANGULAR_VEL:
                angular_z = 0.0

            if abs(error_lin) < DIST_DEADZONE_M:
                linear_x = 0.0
            elif 0.0 < abs(linear_x) < MIN_LINEAR_VEL:
                linear_x = 0.0

            if INVERT_STEERING:
                angular_z = -angular_z

            linear_x  = max(-MAX_LINEAR_VEL,  min(MAX_LINEAR_VEL,  linear_x))
            angular_z = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular_z))

            self.send_cmd_vel(linear_x, angular_z)

    def send_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def send_zero_vel(self):
        self.send_cmd_vel(0.0, 0.0)

    # --- Obsługa Nav2 ---
    def send_nav_goal(self, x, y, yaw):
        # Sprawdzenie czy serwer akcji Nav2 na RPi 4 odpowiada
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().error("Nav2 nie odpowiada! Upewnij się, że RPi 4 działa.")
            return

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        
        # POPRAWKA DLA FIZYCZNEGO ROBOTA: Współrzędne są z /odom, więc frame_id to odom (lub map)!
        pose.header.frame_id = 'odom' 
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        
        q = get_quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal_msg.pose = pose
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state = 'IDLE'
            return
        self.current_goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.current_goal_handle = None
        self.state = 'IDLE'
        self.nav2_failed_time = self.get_clock().now()

    def cancel_current_goal(self):
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        self.state = 'IDLE'

def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()