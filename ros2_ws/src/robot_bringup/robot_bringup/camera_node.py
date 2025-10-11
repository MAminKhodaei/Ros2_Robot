import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

# کتابخانه‌های جدید برای دوربین رزبری پای
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from libcamera import controls

class CameraPublisher(Node):
    def __init__(self):
        # 1. مقداردهی اولیه ROS
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_feed', 10)
        self.bridge = CvBridge()
        self.declare_parameter('camera_id', 0)
        
        # 2. راه‌اندازی Picamera2
        try:
            self.picam2 = Picamera2()
            
            # پیکربندی:
            # - mode: "still" برای کیفیت خوب و پیش‌فرض
            # - size: وضوح تصویر
            # - format: "BGR888" برای سازگاری مستقیم با OpenCV/CV_Bridge
            camera_config = self.picam2.create_video_configuration(
                main={"size": (640, 480), "format": "BGR888"},
                # برای فعالسازی preview و تنظیمات دوربین می‌توانید از control استفاده کنید
                controls={"FrameRate": 30} 
            )
            self.picam2.configure(camera_config)
            
            # شروع ضبط/استریم تصویر
            self.picam2.start()
            self.get_logger().info('Picamera2 started successfully.')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize Picamera2: {e}')
            self.picam2 = None  # تنظیم به None برای جلوگیری از خطا در حلقه اصلی

        # 3. راه‌اندازی تایمر و فرکانس فریم
        # ارسال 30 فریم بر ثانیه
        if self.picam2:
            timer_period = 1.0 / 30.0  
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.get_logger().info('Camera publisher node started.')
        else:
            self.get_logger().error('Camera node shutting down due to Picamera2 failure.')

    def timer_callback(self):
        if self.picam2:
            # 4. گرفتن فریم و انتشار آن
            # capture_array فریم را به صورت یک آرایه numpy به ما می‌دهد
            frame = self.picam2.capture_array() 
            
            if frame is not None:
                # تبدیل فریم OpenCV (NumPy array) به پیام ROS Image و انتشار
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                self.publisher_.publish(ros_image)
            else:
                self.get_logger().warn('Could not capture frame from Picamera2.')
        else:
             # اگر دوربین راه‌اندازی نشد، تایمر را متوقف می‌کنیم.
             if hasattr(self, 'timer'):
                 self.timer.cancel()
                 
    def destroy_node(self):
        # 5. توقف Picamera2 هنگام بسته شدن نود
        if self.picam2:
            self.picam2.stop()
        super().destroy_node()

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