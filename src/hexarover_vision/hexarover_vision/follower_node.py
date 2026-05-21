"""
follower_node.py
----------------
Sterownik PID podążający za człowiekiem.

Subskrybuje:
  /human_angle    (Float32) – kąt człowieka w układzie kamery [stopnie]
                              lewo = +, prawo = –
  /human_distance (Float32) – odległość do człowieka [metry]

Publikuje:
  /cmd_vel (Twist) – komendy prędkości dla Cytron MDDS30
                     linear.x  ∈ [-1.0,  1.0]
                     angular.z ∈ [-1.0,  1.0]

======================================================
PARAMETRY DO TUNINGU – zmieniaj tutaj
======================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# ── Odległość podążania ──────────────────────────────────────────────
DESIRED_DISTANCE_M = 0.5   # robot trzyma się w tej odległości [m]

# ── Limity prędkości ─────────────────────────────────────────────────
MAX_LINEAR_VEL  = 0.35     # maks prędkość liniowa (silniki Dagu max 0.5!)
MAX_ANGULAR_VEL = 0.4      # maks prędkość kątowa

# ── PID kątowy (obrót w kierunku człowieka) ──────────────────────────
# error = angle_deg (kąt z kamery)
# Kp: większa → ostrzejszy skręt; za duża → oscylacje
# Ki: kompensuje stały offset; zacznij od 0.0
# Kd: tłumi oscylacje; zacznij od małej wartości
KP_ANG = 0.012   # zacznij tu, zwiększaj o 0.002 jeśli reaguje za wolno
KI_ANG = 0.0
KD_ANG = 0.003

# ── PID liniowy (utrzymanie odległości) ──────────────────────────────
# error = distance - DESIRED_DISTANCE_M
# Kp: większa → ostrzejsze przyspieszenie; za duża → przejeżdża
# Ki: zacznij od 0.0
# Kd: tłumi przejeżdżanie
KP_LIN = 0.4     # zacznij tu
KI_LIN = 0.0
KD_LIN = 0.05

# ── Strefa martwa ────────────────────────────────────────────────────
# poniżej tego kąta/dystansu – nie kręć/nie jedź (eliminuje drgania)
ANGLE_DEADZONE_DEG  = 5.0   # stopnie
DIST_DEADZONE_M     = 0.08  # metry

# ── Timeout bezpieczeństwa ───────────────────────────────────────────
# jeśli przez ten czas nie ma danych → STOP
SAFETY_TIMEOUT_S = 0.8
# ====================================================


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')

        self.angle_sub = self.create_subscription(
            Float32, '/human_angle',    self.angle_callback,    10)
        self.dist_sub  = self.create_subscription(
            Float32, '/human_distance', self.distance_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ostatnie pomiary
        self.last_angle_deg = 0.0
        self.last_distance  = DESIRED_DISTANCE_M

        # całki i poprzednie błędy dla PID
        self.integral_ang  = 0.0
        self.integral_lin  = 0.0
        self.prev_error_ang = 0.0
        self.prev_error_lin = 0.0

        # timestampy
        self.last_data_time = self.get_clock().now()
        self.last_pid_time  = self.get_clock().now()

        # pętla sterowania 20 Hz
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"Follower gotowy | cel: {DESIRED_DISTANCE_M}m | "
            f"Kp_ang={KP_ANG} Kp_lin={KP_LIN}")

    def angle_callback(self, msg):
        self.last_angle_deg = msg.data
        self.last_data_time = self.get_clock().now()

    def distance_callback(self, msg):
        self.last_distance  = msg.data
        self.last_data_time = self.get_clock().now()

    def control_loop(self):
        now     = self.get_clock().now()
        elapsed = (now - self.last_data_time).nanoseconds / 1e9

        # ── SAFETY: brak danych → STOP ───────────────────────────────
        if elapsed > SAFETY_TIMEOUT_S:
            self.publish_stop()
            self.integral_ang = 0.0
            self.integral_lin = 0.0
            return

        # ── dt dla całkowania i różniczkowania ────────────────────────
        dt = (now - self.last_pid_time).nanoseconds / 1e9
        self.last_pid_time = now
        if dt <= 0.0:
            return

        # ── BŁĘDY ────────────────────────────────────────────────────
        error_ang = self.last_angle_deg                      # [stopnie]
        error_lin = self.last_distance - DESIRED_DISTANCE_M  # [metry]

        # strefa martwa
        if abs(error_ang) < ANGLE_DEADZONE_DEG:
            error_ang = 0.0
        if abs(error_lin) < DIST_DEADZONE_M:
            error_lin = 0.0

        # ── PID kątowy ───────────────────────────────────────────────
        self.integral_ang  += error_ang * dt
        deriv_ang           = (error_ang - self.prev_error_ang) / dt
        self.prev_error_ang = error_ang

        angular_z = (KP_ANG * error_ang +
                     KI_ANG * self.integral_ang +
                     KD_ANG * deriv_ang)

        # ── PID liniowy ──────────────────────────────────────────────
        self.integral_lin  += error_lin * dt
        deriv_lin           = (error_lin - self.prev_error_lin) / dt
        self.prev_error_lin = error_lin

        linear_x = (KP_LIN * error_lin +
                    KI_LIN * self.integral_lin +
                    KD_LIN * deriv_lin)

        # gdy człowiek za blisko – cofaj, ale nie kręć mocno
        if error_lin < -DIST_DEADZONE_M:
            linear_x = max(linear_x, -MAX_LINEAR_VEL * 0.5)

        # ── CLAMP ────────────────────────────────────────────────────
        linear_x  = max(-MAX_LINEAR_VEL,  min(MAX_LINEAR_VEL,  linear_x))
        angular_z = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular_z))

        self.get_logger().info(
            f"PID | ang_err={error_ang:.1f}° lin_err={error_lin:.2f}m "
            f"→ lin={linear_x:.3f} ang={angular_z:.3f}",
            throttle_duration_sec=0.5)

        # ── PUBLISH ──────────────────────────────────────────────────
        cmd = Twist()
        cmd.linear.x  = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()