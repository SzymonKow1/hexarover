import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import socket
import json
import threading

PORT = 5005

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher = self.create_publisher(Imu, '/imu', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', PORT))
        self.get_logger().info('IMU node uruchomiony, czekam na dane...')
        thread = threading.Thread(target=self.receive_loop)
        thread.daemon = True
        thread.start()

    def receive_loop(self):
        while True:
            data, _ = self.sock.recvfrom(1024)
            raw = json.loads(data.decode())
            
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu'
            
            # Przelicz surowe wartości (0-63) na m/s²
            # Środek skali to 32, zakres ±1.5g
            scale = 9.81 * 1.5 / 32.0
            msg.linear_acceleration.x = (raw['x'] - 32) * scale
            msg.linear_acceleration.y = (raw['y'] - 32) * scale
            msg.linear_acceleration.z = (raw['z'] - 32) * scale
            
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()