# -----------------------------------------------------------------------------------
# SRF02 Distance Sensor ROS 2 Node (I2C Communication)
# توجه: این ماژول در نسخه نهایی روبات استفاده نشد.
# در نسخه نهایی، داده‌های فاصله از طریق میکروکنترلر (آردوینو) و پورت سریال دریافت شدند
# (رجوع شود به serial_bridge_node.py).
# -----------------------------------------------------------------------------------

import rclpy # کتابخانه اصلی ROS 2
from rclpy.node import Node # کلاس پایه برای نود ROS
from sensor_msgs.msg import Range # نوع پیام برای انتشار داده‌های سنسور فاصله
from smbus2 import SMBus # کتابخانه برای ارتباط با باس I2C
import time # برای تأخیرهای زمانی بین دستورات I2C

# آدرس I2C پیش‌فرض سنسور SRF02
SRF02_I2C_ADDRESS = 0x70

class SRF02Node(Node):
    """
    این نود مسئول برقراری ارتباط با سنسور SRF02 از طریق I2C،
    دریافت مقدار فاصله و انتشار آن در تاپیک ROS 2 است.
    """
    def __init__(self):
        super().__init__('srf02_distance_sensor_node') # تعیین نام نود
        
        # تعریف Publisher برای انتشار پیام Range در تاپیک 'distance'
        self.publisher_ = self.create_publisher(Range, 'distance', 10)
        
        # تعریف Timer برای اجرای متد read_distance هر ۰.۲ ثانیه (۵ هرتز)
        self.timer = self.create_timer(0.2, self.read_distance)
        
        self.bus = None # شیء باس I2C
        try:
            # تلاش برای باز کردن باس I2C شماره ۱ (معمولاً برای رزبری پای)
            self.bus = SMBus(1)
            self.get_logger().info('SRF02 Node started and I2C bus is open.')
        except Exception as e:
            # مدیریت خطا در صورت عدم موفقیت در باز کردن باس I2C
            self.get_logger().error(f"Failed to open I2C bus: {e}")

    def read_distance(self):
        """
        متد Callback که توسط Timer فراخوانی می‌شود و فاصله را از سنسور می‌خواند.
        """
        if self.bus is None: 
            return # اگر باس I2C باز نشده باشد، از متد خارج می‌شود

        try:
            # گام ۱: ارسال دستور شروع اندازه‌گیری (دستور 0x51 برای فاصله بر حسب سانتی‌متر)
            # نوشتن مقدار 0x51 در رجیستر فرمان (0x00) سنسور
            self.bus.write_byte_data(SRF02_I2C_ADDRESS, 0x00, 0x51)
            
            # گام ۲: کمی صبر کردن تا سنسور اندازه‌گیری را انجام دهد
            time.sleep(0.08) # ۸۰ میلی‌ثانیه برای تکمیل اندازه‌گیری توسط سنسور
            
            # گام ۳: خواندن دو بایت داده (فاصله)
            # خواندن ۲ بایت از رجیستر High Byte (0x02) و Low Byte سنسور
            read_data = self.bus.read_i2c_block_data(SRF02_I2C_ADDRESS, 0x02, 2)
            
            # ترکیب دو بایت برای محاسبه فاصله بر حسب سانتی‌متر
            # read_data[0] (بایت بالا) شیفت ۸ بیتی به چپ داده می‌شود و با read_data[1] (بایت پایین) OR می‌شود
            distance_cm = (read_data[0] << 8) + read_data[1]
            # تبدیل واحد از سانتی‌متر به متر برای استاندارد ROS
            distance_m = float(distance_cm) / 100.0

            # فیلتر کردن مقادیر غیرمنطقی
            if distance_m <= 0.0:
                self.get_logger().warn("Distance is zero or negative, skipping measurement.")
                return

            # ------------------- ساخت و انتشار پیام Range -------------------
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg() # زمان‌بندی پیام
            msg.header.frame_id = 'ultrasonic_sensor_link' # فریم سنسور
            msg.radiation_type = Range.ULTRASOUND # نوع تابش (التراسونیک)
            msg.field_of_view = 0.2 # زاویه دید (رادیان)
            msg.min_range = 0.15 # حداقل برد قابل اطمینان سنسور (متر)
            msg.max_range = 6.0 # حداکثر برد سنسور (متر)
            msg.range = distance_m # مقدار فاصله اندازه‌گیری شده (متر)
            
            self.publisher_.publish(msg) # انتشار پیام
            
        except Exception as e:
            self.get_logger().warn(f"Failed to read from SRF02 sensor: {e}") # مدیریت خطاهای I2C

def main(args=None):
    rclpy.init(args=args) # مقداردهی اولیه ROS 2
    srf02_node = SRF02Node() # ایجاد نود
    rclpy.spin(srf02_node) # اجرای حلقه اصلی نود
    srf02_node.destroy_node() # پاکسازی نود
    rclpy.shutdown() # خاموش کردن ROS 2

if __name__ == '__main__':
    main()
