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
        
        # هر 0.1 ثانیه (10 فریم بر ثانیه) تابع زیر را اجرا کن
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # دسترسی به دوربین (معمولاً با ایندکس 0)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video capture device")
        
        self.bridge = CvBridge()
        self.get_logger().info('Camera Node has been started.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # تبدیل فریم OpenCV به پیام Image در ROS
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(ros_image_msg)
        # else:
        #     self.get_logger().warn("Could not read frame from camera")

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()