import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import time

I2C_BUS      = 1
MMA7660_ADDR = 0x4c
SCALE        = 9.81 * 1.5 / 32.0

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher = self.create_publisher(Imu, '/imu', 10)
        self.bus = smbus2.SMBus(I2C_BUS)

        # Włączenie trybu aktywnego (rejestr 0x07 = MODE, wartość 0x01 = Active)
        self.bus.write_byte_data(MMA7660_ADDR, 0x07, 0x00)  # standby
        time.sleep(0.1)
        self.bus.write_byte_data(MMA7660_ADDR, 0x07, 0x01)  # active
        time.sleep(0.1)

        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('IMU node uruchomiony, czytam z I2C...')

    def timer_callback(self):
        try:
            x = self.bus.read_byte_data(MMA7660_ADDR, 0x00)
            y = self.bus.read_byte_data(MMA7660_ADDR, 0x01)
            z = self.bus.read_byte_data(MMA7660_ADDR, 0x02)

            # Bit 6 to flaga alertu - ignorujemy taki odczyt
            if (x & 0x40) or (y & 0x40) or (z & 0x40):
                return

            # Konwersja z 6-bitowej liczby ze znakiem
            x = x & 0x3F
            y = y & 0x3F
            z = z & 0x3F

            if x > 31: x -= 64
            if y > 31: y -= 64
            if z > 31: z -= 64

            self.get_logger().info(f'Raw: x={x} y={y} z={z}', throttle_duration_sec=1.0)

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'
            msg.linear_acceleration.x = x * SCALE
            msg.linear_acceleration.y = y * SCALE
            msg.linear_acceleration.z = z * SCALE

            msg.orientation_covariance[0]         = -1.0
            msg.angular_velocity_covariance[0]    = -1.0
            msg.linear_acceleration_covariance[0] = 0.1

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().warning(f'Błąd odczytu I2C: {e}', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
