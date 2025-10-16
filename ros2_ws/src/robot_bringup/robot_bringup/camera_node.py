# -----------------------------------------------------------------------------------
# Camera Publisher Node (Picamera2 - Raspberry Pi Camera Module)
# توجه: این نود برای کار با دوربین اختصاصی رزبری پای (CSI/DSI) طراحی شده بود.
# در نسخه نهایی روبات، به دلیل مشکلات سازگاری/لجستیکی، این نود کنار گذاشته شد
# و به جای آن از درایور استاندارد ROS 2 برای وب‌کم USB (v4l2_camera) استفاده شد.
# -----------------------------------------------------------------------------------

import rclpy # کتابخانه اصلی ROS 2
from rclpy.node import Node # کلاس پایه نود
from sensor_msgs.msg import Image # نوع پیام ROS برای انتشار تصاویر
from cv_bridge import CvBridge # ابزاری برای تبدیل تصاویر OpenCV/NumPy به پیام‌های ROS
import numpy as np # کتابخانه NumPy برای کار با آرایه‌های داده‌ای (تصاویر)

# کتابخانه‌های مخصوص مدیریت دوربین‌های نسل جدید رزبری پای (Picamera2)
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder # ابزار رمزگذاری
from libcamera import controls # ابزار تنظیم کنترل‌های سخت‌افزاری دوربین

class CameraPublisher(Node):
    """
    این نود تلاش می‌کند فریم‌های تصویر را از طریق Picamera2 دریافت کرده
    و آن‌ها را در قالب پیام ROS Image در یک تاپیک منتشر کند.
    """
    def __init__(self):
        # ۱. مقداردهی اولیه ROS
        super().__init__('camera_publisher') # تعیین نام نود
        self.publisher_ = self.create_publisher(Image, 'image_feed', 10) # ایجاد Publisher برای تاپیک 'image_feed'
        self.bridge = CvBridge() # ایجاد نمونه‌ای از CvBridge برای تبدیل داده‌های تصویر
        self.declare_parameter('camera_id', 0) # تعریف پارامتر ID دوربین (در این ساختار استفاده نشد اما باقی می‌ماند)
        
        # ۲. راه‌اندازی Picamera2
        try:
            self.picam2 = Picamera2() # ایجاد نمونه Picamera2
            
            # پیکربندی دوربین:
            camera_config = self.picam2.create_video_configuration(
                # تعیین وضوح و فرمت تصویر (640x480 در فرمت BGR888 برای سازگاری با OpenCV)
                main={"size": (640, 480), "format": "BGR888"}, 
                # تنظیمات کنترلی دوربین، مثلاً نرخ فریم (FPS)
                controls={"FrameRate": 30} 
            )
            self.picam2.configure(camera_config) # اعمال پیکربندی
            
            # شروع ضبط/استریم تصویر
            self.picam2.start()
            self.get_logger().info('Picamera2 started successfully.')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize Picamera2: {e}')
            self.picam2 = None  # تنظیم به None برای جلوگیری از خطا در حلقه اصلی

        # ۳. راه‌اندازی تایمر و فرکانس فریم
        # اگر دوربین با موفقیت راه‌اندازی شد، تایمر را تنظیم می‌کنیم
        if self.picam2:
            timer_period = 1.0 / 30.0 # محاسبه فاصله زمانی برای ارسال ۳۰ فریم بر ثانیه
            self.timer = self.create_timer(timer_period, self.timer_callback) # ایجاد تایمر
            self.get_logger().info('Camera publisher node started.')
        else:
            self.get_logger().error('Camera node shutting down due to Picamera2 failure.')

    def timer_callback(self):
        """
        متد Callback تایمر که فریم‌ها را در فرکانس مشخص شده (۳۰ هرتز) دریافت و منتشر می‌کند.
        """
        if self.picam2:
            # ۴. گرفتن فریم و انتشار آن
            # capture_array فریم را به صورت یک آرایه NumPy (OpenCV format) دریافت می‌کند
            frame = self.picam2.capture_array() 
            
            if frame is not None:
                # تبدیل فریم NumPy (OpenCV) به پیام استاندارد ROS Image
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                # زمان‌بندی پیام
                ros_image.header.stamp = self.get_clock().now().to_msg()
                self.publisher_.publish(ros_image) # انتشار پیام
            else:
                self.get_logger().warn('Could not capture frame from Picamera2.')
        else:
             # اگر دوربین راه‌اندازی نشد، تایمر را متوقف می‌کنیم.
             if hasattr(self, 'timer'):
                 self.timer.cancel()
                 
    def destroy_node(self):
        """
        پاکسازی منابع هنگام بسته شدن نود.
        """
        # ۵. توقف Picamera2 هنگام بسته شدن نود
        if self.picam2:
            self.picam2.stop()
        super().destroy_node() # فراخوانی متد destroy_node کلاس والد

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()