# serial_bridge_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
import serial
import threading
import time

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # پارامترها برای پورت سریال
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        # تلاش برای اتصال به پورت سریال
        try:
            self.serial_conn = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"موفقیت در اتصال به آردوینو روی پورت {port}")
        except Exception as e:
            self.get_logger().error(f"خطا در اتصال به آردوینو: {e}")
            return

        # Subscriber برای دریافت دستورات حرکتی
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publisher برای ارسال داده‌های GPS
        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)

        # ایجاد یک ترد جداگانه برای خواندن مداوم از پورت سریال
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

    def cmd_vel_callback(self, msg):
        """تبدیل پیام Twist به دستور تک-کاراکتری و ارسال به آردوینو"""
        command = 'S' # دستور پیش‌فرض: توقف
        if msg.linear.x > 0.1:
            command = 'F' # Forward
        elif msg.linear.x < -0.1:
            command = 'B' # Backward
        elif msg.angular.z > 0.1:
            command = 'L' # Left
        elif msg.angular.z < -0.1:
            command = 'R' # Right

        self.serial_conn.write(command.encode())
        self.get_logger().info(f"ارسال دستور '{command}' به آردوینو")

    def serial_read_loop(self):
        """حلقه خواندن داده از آردوینو و انتشار آن به عنوان پیام GPS"""
        while rclpy.ok():
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line.startswith('G,'):
                    parts = line.split(',')
                    if len(parts) == 3:
                        lat = float(parts[1])
                        lon = float(parts[2])

                        gps_msg = NavSatFix()
                        gps_msg.header.stamp = self.get_clock().now().to_msg()
                        gps_msg.header.frame_id = 'gps_link'
                        gps_msg.latitude = lat
                        gps_msg.longitude = lon
                        gps_msg.status.status = NavSatFix.STATUS_FIX
                        gps_msg.status.service = NavSatFix.SERVICE_GPS

                        self.gps_pub.publish(gps_msg)
            except Exception as e:
                self.get_logger().warn(f"خطا در خواندن از سریال: {e}")
            time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()