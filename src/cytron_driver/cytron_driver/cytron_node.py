import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

MAX_SPEED = 63
PORT = '/dev/ttyUSB0'
BAUD = 9600

class CytronDriver(Node):
    def __init__(self):
        super().__init__('cytron_driver')
        self.serial = serial.Serial(PORT, BAUD)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Cytron driver uruchomiony')

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x   # przód linear>0, tyl<0, zakres -1.0 do 1.0 
        angular = msg.angular.z  # skręt w lewo angular>0, prawo<0, zakres -1.0 do 1.0 

        left_speed = linear - angular
        right_speed = linear + angular

        self.send_motor(left=left_speed, right=right_speed)

    def send_motor(self, left, right):
        left_byte = self.to_byte(left, channel=0, reverse=True)
        right_byte = self.to_byte(right, channel=1, reverse=False)
        self.serial.write(bytes([left_byte, right_byte]))

    def to_byte(self, value, channel, reverse=False):
        value = max(-1.0, min(1.0, value))
        direction = 0 if value >= 0 else 1
        if reverse:
            direction = 1 - direction
        speed = int(abs(value) * MAX_SPEED)
        return (channel << 7) | (direction << 6) | speed

def main(args=None):
    rclpy.init(args=args)
    node = CytronDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()