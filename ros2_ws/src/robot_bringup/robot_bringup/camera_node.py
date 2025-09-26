import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'video_stream', 10)
        self.bridge = CvBridge()

        # لوله: گرفتن YUYV و تبدیل به BGR توسط videoconvert
        gst_pipeline = (
            "v4l2src device=/dev/video0 ! "
            "video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! "
            "videoconvert ! "
            "appsink"
        )

        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("❌ Could not open /dev/video0 with pipeline.")
        else:
            self.get_logger().info("✅ Camera opened successfully with YUYV→BGR conversion.")

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret and frame is not None:
            # OpenCV حالا خودش BGR برمی‌گردونه
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("⚠️ Could not read frame from /dev/video0")

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap.isOpened():
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
