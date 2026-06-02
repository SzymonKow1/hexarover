import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

# ==========================================
# STAŁE KONFIGURACYJNE
# ==========================================
MAX_SPEED_RATIO = 7.5 / 11.1   # Ograniczenie napięcia silników (max 7.5V, bateria 11.1V)
PACKETIZED_ADDRESS = 0          # Adres sterownika (SW4-SW6 = 000)
PORT = '/dev/cytron'
BAUD = 9600                     # Baudrate do autobaud — musi zgadzać się z tym co wysyłamy

HEADER = 0x55                   # Stały nagłówek pakietu Serial Packetized
CHANNEL_LEFT  = 0               # bit3=0 → lewy silnik
CHANNEL_RIGHT = 1               # bit3=1 → prawy silnik

# Komenda 127 = stop, 0 = full reverse, 255 = full forward
CMD_STOP = 127

# ==========================================

def make_packet(channel, command, address=PACKETIZED_ADDRESS):
    """Buduje 4-bajtowy pakiet Serial Packetized."""
    channel_byte = (channel << 3) | (address & 0x07)
    checksum = (HEADER + channel_byte + command) % 256
    return bytes([HEADER, channel_byte, command, checksum])

def speed_to_cmd(value, reverse=False):
    """
    Mapuje wartość -1.0..1.0 na komendę 0..255.
    127 = stop, 255 = full forward, 0 = full reverse.
    reverse=True odwraca kierunek (dla lewego silnika).
    """
    value = max(-1.0, min(1.0, value))
    value *= MAX_SPEED_RATIO  # Ograniczenie napięcia
    if reverse:
        value = -value
    # Mapowanie: 0.0 → 127, 1.0 → 255, -1.0 → 0
    cmd = int(127 + value * 127)
    return max(0, min(255, cmd))

class CytronDriver(Node):
    def __init__(self):
        super().__init__('cytron_driver')
        self.serial = serial.Serial(PORT, BAUD)

        # Autobaud: wyślij 0x80 żeby Cytron wykrył baudrate
        self.serial.write(bytes([0x80]))
        time.sleep(0.1)  # Chwila na wykrycie baudrate

        # Wyślij STOP na oba silniki od razu
        self.serial.write(make_packet(CHANNEL_LEFT,  CMD_STOP))
        self.serial.write(make_packet(CHANNEL_RIGHT, CMD_STOP))

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Cytron driver uruchomiony (Serial Packetized)')

    def cmd_vel_callback(self, msg):
        linear  = msg.linear.x   # przód >0, tył <0, zakres -1.0 do 1.0
        angular = msg.angular.z  # lewo >0, prawo <0, zakres -1.0 do 1.0

        left_speed  = linear - angular
        right_speed = linear + angular

        # Lewy silnik odwrócony (reverse=True) — tak samo jak w poprzedniej wersji
        left_cmd  = speed_to_cmd(left_speed,  reverse=True)
        right_cmd = speed_to_cmd(right_speed, reverse=False)

        self.serial.write(make_packet(CHANNEL_LEFT,  left_cmd))
        self.serial.write(make_packet(CHANNEL_RIGHT, right_cmd))

def main(args=None):
    rclpy.init(args=args)
    node = CytronDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()