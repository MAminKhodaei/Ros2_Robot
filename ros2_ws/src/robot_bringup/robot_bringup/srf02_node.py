# srf02_node.py - نسخه پایدارتر
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from smbus2 import SMBus
import time

SRF02_I2C_ADDRESS = 0x70

class SRF02Node(Node):
    def __init__(self):
        super().__init__('srf02_distance_sensor_node')
        self.publisher_ = self.create_publisher(Range, 'distance', 10)
        self.timer = self.create_timer(0.2, self.read_distance)
        self.bus = None
        try:
            self.bus = SMBus(1)
            self.get_logger().info('SRF02 Node started and I2C bus is open.')
        except Exception as e:
            self.get_logger().error(f"Failed to open I2C bus: {e}")

    def read_distance(self):
        if self.bus is None: return

        try:
            # گام ۱: ارسال دستور شروع اندازه‌گیری (فاصله بر حسب سانتی‌متر)
            # از دستور write_byte_data برای سادگی و پایداری بیشتر استفاده می‌کنیم
            self.bus.write_byte_data(SRF02_I2C_ADDRESS, 0x00, 0x51)
            
            # گام ۲: کمی بیشتر صبر می‌کنیم تا سنسور کارش را تمام کند
            time.sleep(0.08) # افزایش زمان انتظار به ۸۰ میلی‌ثانیه
            
            # گام ۳: خواندن دو بایت داده از رجیستر شماره ۲
            read_data = self.bus.read_i2c_block_data(SRF02_I2C_ADDRESS, 0x02, 2)
            
            distance_cm = (read_data[0] << 8) + read_data[1]
            distance_m = float(distance_cm) / 100.0

            # اگر فاصله صفر بود، آن را منتشر نکن (احتمالاً خطای سنسور است)
            if distance_m <= 0.0:
                self.get_logger().warn("Distance is zero, skipping measurement.")
                return

            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'ultrasonic_sensor_link'
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.2
            msg.min_range = 0.15
            msg.max_range = 6.0
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