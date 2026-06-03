import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

# ==========================================
# STAŁE KONFIGURACYJNE
# ==========================================
MAX_SPEED_RATIO = 7.5 / 11.1   # Ograniczenie napięcia silników
PACKETIZED_ADDRESS = 0          # Adres sterownika (SW4-SW6 = 000)
PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
BAUD = 9600                     

HEADER = 0x55                   # Nagłówek pakietu
CHANNEL_LEFT  = 0               # Lewy silnik
CHANNEL_RIGHT = 1               # Prawy silnik

CMD_STOP = 127
# ==========================================

def make_packet(channel, command, address=PACKETIZED_ADDRESS):
    channel_byte = (channel << 3) | (address & 0x07)
    checksum = (HEADER + channel_byte + command) % 256
    return bytes([HEADER, channel_byte, command, checksum])

def speed_to_cmd(value, reverse=False):
    value = max(-1.0, min(1.0, value))
    value *= MAX_SPEED_RATIO
    if reverse:
        value = -value
    cmd = int(127 + value * 127)
    return max(0, min(255, cmd))


class CytronDriver(Node):
    def __init__(self):
        super().__init__('cytron_driver')
        
        try:
            self.serial = serial.Serial(PORT, BAUD, timeout=1.0)
        except Exception as e:
            self.get_logger().error(f"Nie można otworzyć portu Cytrona {PORT}: {e}")
            return

        # Autobaud: wyślij 0x80 na początku
        self.serial.write(bytes([0x80]))
        time.sleep(1.0)  # Dajemy Cytronowi czas na zsynchronizowanie baudrate

        # Domyślne wartości poleceń (STOP)
        self.left_cmd = CMD_STOP
        self.right_cmd = CMD_STOP

        # Subskrypcja cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # KLUCZOWA ZMIANA: Timer działający z częstotliwością 20 Hz (co 50 ms)
        # Będzie nieprzerwanie wysyłał dane do Cytrona, aby zapobiec aktywacji 100 ms timeoutu
        self.timer = self.create_timer(0.05, self.send_commands_callback)

        self.get_logger().info('Cytron driver gotowy i zabezpieczony przed Timeoutem!')

    def cmd_vel_callback(self, msg):
        linear  = msg.linear.x   
        angular = msg.angular.z  

        left_speed  = linear - angular
        right_speed = linear + angular

        # Aktualizujemy stany (wysyłaniem zajmuje się timer)
        self.left_cmd  = speed_to_cmd(left_speed,  reverse=True)
        self.right_cmd = speed_to_cmd(right_speed, reverse=False)

    def send_commands_callback(self):
        """Wysyła aktualnie zapamiętane prędkości do silników co 50 ms."""
        try:
            self.serial.write(make_packet(CHANNEL_LEFT,  self.left_cmd))
            self.serial.write(make_packet(CHANNEL_RIGHT, self.right_cmd))
        except Exception as e:
            self.get_logger().warning(f"Błąd wysyłania danych do Cytrona: {e}", throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = CytronDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
