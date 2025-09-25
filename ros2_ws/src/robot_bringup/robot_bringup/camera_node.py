# camera_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'video_stream', 10)
        
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # استفاده از GStreamer Pipeline برای دسترسی به دوربین CSI
        # توجه: این خط لوله برای دوربین های Pi Cam v1 و v2 به خوبی کار می کند.
        # می‌توانید رزولوشن را از 640x480 به 1280x720 تغییر دهید.
        gst_pipeline = (
            "libcamerasrc ! "
            "video/x-raw, width=640, height=480, framerate=30/1 ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink"
        )
        
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video capture device with GStreamer pipeline.")
            # می‌توانید پیام خطای دقیق‌تری نمایش دهید
            self.get_logger().error("Check if libcamera and GStreamer are correctly installed.")
            
        self.bridge = CvBridge()
        self.get_logger().info('Camera Node has been started.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(ros_image_msg)
        else:
            self.get_logger().warn("Could not read frame from camera. Is it connected and working?")

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        # حتما منابع را آزاد کنید
        camera_node.cap.release()
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()