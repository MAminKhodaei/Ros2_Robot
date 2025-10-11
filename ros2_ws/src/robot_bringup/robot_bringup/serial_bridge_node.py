
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Range 
import serial
import threading
import time

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        try:
            self.serial_conn = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"موفقیت در اتصال به آردوینو روی پورت {port}")
        except Exception as e:
            self.get_logger().error(f"خطا در اتصال به آردوینو: {e}")
            rclpy.shutdown()
            return

        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.distance_pub = self.create_publisher(Range, 'distance', 10)
        
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

    def cmd_vel_callback(self, msg):
        command = 'S'
        if msg.linear.x > 0.1: command = 'F'
        elif msg.linear.x < -0.1: command = 'B'
        elif msg.angular.z > 0.1: command = 'L'
        elif msg.angular.z < -0.1: command = 'R'
        self.serial_conn.write(command.encode())

    def serial_read_loop(self):
        while rclpy.ok():
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                
                if line.startswith('G,'):
                    parts = line.split(',')
                    if len(parts) == 3:
                        lat, lon = float(parts[1]), float(parts[2])
                        gps_msg = NavSatFix()
                        gps_msg.header.stamp = self.get_clock().now().to_msg()
                        gps_msg.header.frame_id = 'gps_link'
                        gps_msg.latitude, gps_msg.longitude = lat, lon
                        self.gps_pub.publish(gps_msg)

                elif line.startswith('D,'):
                    parts = line.split(',')
                    if len(parts) == 2:
                        distance_m = float(parts[1]) / 100.0
                        range_msg = Range()
                        range_msg.header.stamp = self.get_clock().now().to_msg()
                        range_msg.header.frame_id = 'ultrasonic_sensor_link'
                        range_msg.radiation_type = Range.ULTRASOUND
                        range_msg.field_of_view = 0.26
                        range_msg.min_range = 0.02
                        range_msg.max_range = 4.0
                        range_msg.range = distance_m
                        self.distance_pub.publish(range_msg)
                            
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