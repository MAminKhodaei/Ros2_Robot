# -----------------------------------------------------------------------------------
# وارد کردن کتابخانه‌ها
# -----------------------------------------------------------------------------------
import rclpy # کتابخانه اصلی ROS 2 برای برنامه‌نویسی پایتون
from rclpy.node import Node # کلاس پایه برای ساخت نودهای ROS 2
from geometry_msgs.msg import Twist # نوع پیام ROS برای فرمان‌های سرعت (حرکت خطی و چرخشی)
from sensor_msgs.msg import NavSatFix, Range # نوع پیام برای داده‌های GPS (NavSatFix) و سنسور فاصله/التراسونیک (Range)
import serial # کتابخانه PySerial برای مدیریت ارتباطات سریال (مثلاً با آردوینو)
import threading # برای اجرای حلقه خواندن سریال در یک رشته (Thread) جداگانه و غیربلاک کننده
import time # برای ایجاد تأخیرهای زمانی در حلقه خواندن

# -----------------------------------------------------------------------------------
# تعریف کلاس اصلی نود: SerialBridgeNode
# -----------------------------------------------------------------------------------
class SerialBridgeNode(Node): # تعریف کلاس نود با وراثت از rclpy.node.Node
    def __init__(self):
        super().__init__('serial_bridge_node') # فراخوانی سازنده والد و تعیین نام نود در ROS 2

        # ------------------- تعریف و دریافت پارامترها -------------------
        # تعریف پارامترهایی که می‌توانند از بیرون (مثل فایل launch) تنظیم شوند
        self.declare_parameter('serial_port', '/dev/ttyUSB0') # تعریف پورت سریال (پیش‌فرض: /dev/ttyUSB0)
        self.declare_parameter('baud_rate', 115200) # تعریف نرخ انتقال داده (Baud Rate) (پیش‌فرض: 115200)

        # دریافت مقادیر پارامترهای تعریف شده
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        # ------------------- برقراری اتصال سریال -------------------
        try:
            # تلاش برای برقراری اتصال سریال با تنظیمات مشخص شده و زمان‌بندی (timeout) ۱ ثانیه
            self.serial_conn = serial.Serial(port, baud, timeout=1)
            # ثبت پیام موفقیت‌آمیز اتصال در لاگر ROS 2
            self.get_logger().info(f"موفقیت در اتصال به آردوینو روی پورت {port}")
        except Exception as e:
            # مدیریت خطا در صورت عدم موفقیت در اتصال سریال
            self.get_logger().error(f"خطا در اتصال به آردوینو: {e}")
            rclpy.shutdown() # خاموش کردن ROS 2 در صورت شکست اتصال بحرانی
            return # خروج از تابع سازنده

        # ------------------- تعریف Subscriberها و Publisherها -------------------
        # ایجاد Subscriber برای دریافت پیام‌های Twist در تاپیک 'cmd_vel'
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        # ایجاد Publisher برای انتشار داده‌های GPS در تاپیک 'gps/fix'
        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        # ایجاد Publisher برای انتشار داده‌های سنسور فاصله/رنج در تاپیک 'distance'
        self.distance_pub = self.create_publisher(Range, 'distance', 10)
        
        # ------------------- راه‌اندازی رشته (Thread) خواندن سریال -------------------
        # ایجاد یک رشته جداگانه برای اجرای حلقه خواندن سریال (serial_read_loop)
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        # daemon=True تضمین می‌کند که رشته با بسته شدن نود اصلی بسته شود
        self.read_thread.start() # شروع به کار رشته

    # -----------------------------------------------------------------------------------
    # متد Callback برای پردازش پیام‌های Twist
    # -----------------------------------------------------------------------------------
    def cmd_vel_callback(self, msg):
        command = 'S' # دستور پیش‌فرض: توقف (Stop)
        # بررسی سرعت خطی (linear.x) برای تعیین حرکت جلو یا عقب
        if msg.linear.x > 0.1: command = 'F' # حرکت به جلو (Forward)
        elif msg.linear.x < -0.1: command = 'B' # حرکت به عقب (Backward)
        # بررسی سرعت چرخشی (angular.z) برای تعیین چرخش چپ یا راست
        elif msg.angular.z > 0.1: command = 'L' # چرخش به چپ (Turn Left)
        elif msg.angular.z < -0.1: command = 'R' # چرخش به راست (Turn Right)
        # ارسال فرمان تک کاراکتری (F, B, L, R, S) به صورت بایت به پورت سریال
        self.serial_conn.write(command.encode())

    # -----------------------------------------------------------------------------------
    # حلقه اصلی خواندن داده‌ها از پورت سریال
    # -----------------------------------------------------------------------------------
    def serial_read_loop(self):
        while rclpy.ok(): # تکرار تا زمانی که نود ROS 2 فعال باشد
            try:
                # خواندن یک خط کامل از سریال، دیکد کردن به UTF-8 و حذف فاصله‌های ابتدا و انتها
                line = self.serial_conn.readline().decode('utf-8').strip()
                
                # --------------- پردازش داده‌های GPS (G,) ---------------
                if line.startswith('G,'):
                    parts = line.split(',') # جدا کردن داده‌ها با کاما (پروتکل: G,lat,lon)
                    if len(parts) == 3: # بررسی صحت فرمت
                        # تبدیل عرض و طول جغرافیایی به عدد اعشاری
                        lat, lon = float(parts[1]), float(parts[2])
                        
                        gps_msg = NavSatFix() # ایجاد پیام استاندارد NavSatFix
                        # ثبت زمان فعلی سیستم در هدر پیام
                        gps_msg.header.stamp = self.get_clock().now().to_msg()
                        gps_msg.header.frame_id = 'gps_link' # تعیین فریم مرجع
                        # اختصاص مقادیر
                        gps_msg.latitude, gps_msg.longitude = lat, lon
                        self.gps_pub.publish(gps_msg) # انتشار پیام GPS

                # --------------- پردازش داده‌های سنسور فاصله (D,) ---------------
                elif line.startswith('D,'):
                    parts = line.split(',') # جدا کردن داده‌ها با کاما (پروتکل: D,distance_cm)
                    if len(parts) == 2: # بررسی صحت فرمت
                        # تبدیل فاصله (فرض بر سانتی‌متر) به متر (تقسیم بر ۱۰۰)
                        distance_m = float(parts[1]) / 100.0
                        
                        range_msg = Range() # ایجاد پیام استاندارد Range
                        # ثبت زمان
                        range_msg.header.stamp = self.get_clock().now().to_msg()
                        range_msg.header.frame_id = 'ultrasonic_sensor_link' # تعیین فریم مرجع سنسور
                        range_msg.radiation_type = Range.ULTRASOUND # تعیین نوع سنسور
                        range_msg.field_of_view = 0.26 # تعیین زاویه دید سنسور (رادیان)
                        range_msg.min_range = 0.02 # حداقل برد سنسور (متر)
                        range_msg.max_range = 4.0 # حداکثر برد سنسور (متر)
                        range_msg.range = distance_m # اختصاص مقدار فاصله (متر)
                        self.distance_pub.publish(range_msg) # انتشار پیام Range
                            
            except Exception as e:
                # مدیریت و ثبت هشدار در صورت بروز خطا در خواندن یا تجزیه داده
                self.get_logger().warn(f"خطا در خواندن از سریال: {e}")
            # ایجاد تأخیر کوتاه برای جلوگیری از مصرف زیاد CPU (به رشته‌های دیگر اجازه اجرا می‌دهد)
            time.sleep(0.01)

# -----------------------------------------------------------------------------------
# تابع اصلی اجرای برنامه
# -----------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args) # مقداردهی اولیه محیط ROS 2
    node = SerialBridgeNode() # ایجاد نمونه‌ای از نود
    rclpy.spin(node) # وارد کردن نود به حلقه اصلی پردازش (نود را فعال نگه می‌دارد)
    
    # دستورات زیر پس از پایان حلقه spin (مثلاً با فشردن Ctrl+C) اجرا می‌شوند
    node.destroy_node() # پاکسازی و تخریب نود
    rclpy.shutdown() # خاموش کردن ROS 2

# -----------------------------------------------------------------------------------
# نقطه شروع اجرای اسکریپت
# -----------------------------------------------------------------------------------
if __name__ == '__main__':
    main()
