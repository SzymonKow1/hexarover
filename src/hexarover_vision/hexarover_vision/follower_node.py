import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

ROBOT_FRONT_OFFSET = 0.5  # Zmień to na rzeczywistą odległość od środka do przodu zderzaka z URDF
STOP_DIST = 1.0  # Zatrzymaj się 1 metr od człowieka
TOTAL_STOP_DIST = STOP_DIST + ROBOT_FRONT_OFFSET
MAX_DIST= 10.0   # Ignoruj cele dalej niż 5 metrów (błędy lasera)


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.sub_dist = self.create_subscription(Float32, '/human_distance', self.dist_callback, 10)
        self.sub_angle = self.create_subscription(Float32, '/human_angle', self.angle_callback, 10)

        self.latest_dist = None
        self.latest_angle_deg = None
        
        self.current_goal_handle = None
        
        self.state = 'IDLE'

        self.locked_dist = 0.0
        self.locked_angle = 0.0
        
        self.timer = self.create_timer(1.5, self.decision_loop)
        self.get_logger().info("Follower Node V4 (Z blokadą celu) gotowy!")

    def dist_callback(self, msg):
        self.latest_dist = msg.data

    def angle_callback(self, msg):
        self.latest_angle_deg = msg.data

    def decision_loop(self):
        if self.latest_dist is None or self.latest_angle_deg is None:
            return

        dist = self.latest_dist
        angle_rad = math.radians(self.latest_angle_deg)

        if dist > MAX_DIST:
            if self.state != 'IDLE':
                self.cancel_current_goal()
            return

        # 1. Za blisko - Cofanie
        if dist < STOP_DIST:
            if self.state != 'REVERSING':
                self.cancel_current_goal() # Anulujemy pościg, jeśli trwał
                self.get_logger().info("Za blisko! Wycofuję się!")
                self.send_nav_goal(-0.5, 0.0, 0.0)
                self.state = 'REVERSING'
            return

        # 2. Strefa idealna - HAMOWANIE
        if STOP_DIST <= dist <= TOTAL_STOP_DIST:
            if self.state != 'IDLE':
                self.get_logger().info(f"Dystans OK ({dist:.2f}m). HAMUJĘ!")
                self.cancel_current_goal()
                self.state = 'IDLE'
            return

        # 3. Pościg 
        if self.state != 'CHASING':
            # Na wypadek, gdyby robot wcześniej cofał, najpierw czyścimy cel
            self.cancel_current_goal()
            
            drive_dist = dist - TOTAL_STOP_DIST
            target_x = drive_dist * math.cos(angle_rad)
            target_y = drive_dist * math.sin(angle_rad)

            self.get_logger().info(f"Ruszam do celu: {drive_dist:.2f}m przed robota.")
            self.send_nav_goal(target_x, target_y, angle_rad)
            self.state = 'CHASING'
            
            self.locked_dist = dist
            self.locked_angle = self.latest_angle_deg
        else:
            dist_diff = abs(dist - self.locked_dist)
            angle_diff = abs(self.latest_angle_deg - self.locked_angle)

            # Zmieniłem tolerancję kąta, 50 stopni to było bardzo dużo!
            if dist_diff > 0.5 or angle_diff > 15.0:
                self.get_logger().info(f"Człowiek zmienił pozycję! Przeliczam...")
                self.cancel_current_goal() 
                self.state = 'IDLE' # Zresetowanie stanu wymusi wysłanie nowego celu w następnym obrocie

    def send_nav_goal(self, x, y, yaw):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            return

        goal_msg = NavigateToPose.Goal()
        
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
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
        self.state = 'IDLE'
        self.current_goal_handle = None

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