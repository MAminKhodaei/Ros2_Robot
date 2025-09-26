# srf02_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from smbus2 import SMBus
import time

SRF02_I2C_ADDRESS = 0x70  # Default I2C address for SRF02

class SRF02Node(Node):
    def __init__(self):
        super().__init__('srf02_distance_sensor_node')
        self.publisher_ = self.create_publisher(Range, 'distance', 10)

        # Create a timer to read the sensor periodically (5 times a second)
        self.timer = self.create_timer(0.2, self.read_distance)

        try:
            self.bus = SMBus(1)  # 1 indicates /dev/i2c-1
            self.get_logger().info('SRF02 Node started and I2C bus is open.')
        except Exception as e:
            self.get_logger().error(f"Failed to open I2C bus: {e}")
            self.bus = None

    def read_distance(self):
        if self.bus is None:
            return

        try:
            # Command to start ranging in centimeters
            write_data = [0x00, 0x51]
            self.bus.write_i2c_block_data(SRF02_I2C_ADDRESS, 0, write_data[1:])

            # Wait for ranging to complete (SRF02 takes about 65-70ms)
            time.sleep(0.07)

            # Read the 2-byte distance value (high byte, low byte)
            read_data = self.bus.read_i2c_block_data(SRF02_I2C_ADDRESS, 2, 2)

            # Combine the bytes to get the distance and convert to meters for ROS standard
            distance_cm = (read_data[0] << 8) + read_data[1]
            distance_m = float(distance_cm) / 100.0

            # Create and publish the ROS Range message
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'ultrasonic_sensor_link'
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.2  # Approximate field of view in radians
            msg.min_range = 0.15  # 15 cm in meters
            msg.max_range = 6.0   # 6 m in meters
            msg.range = distance_m

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Failed to read from SRF02 sensor: {e}")

def main(args=None):
    rclpy.init(args=args)
    srf02_node = SRF02Node()
    rclpy.spin(srf02_node)
    srf02_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()