# =================================================================
# ==        main.py - نسخه نهایی و صحیح با فرمت رنگ RGB8          ==
# =================================================================
import socketio, rclpy, threading, asyncio, base64, cv2, os
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Range

class RosBridgeNode(Node):
    def __init__(self, sio_server):
        super().__init__('web_gui_ros_bridge')
        self.sio = sio_server
        self.bridge = CvBridge()
        self.cmd_vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gps_subscriber_ = self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.distance_subscriber_ = self.create_subscription(Range, 'distance', self.distance_callback, 10)
        self.image_subscriber_ = self.create_subscription(Image, 'video_stream', self.image_callback, 10)
        self.get_logger().info('ROS 2 Bridge Node is ready.')

    def distance_callback(self, msg):
        distance_cm = round(msg.range * 100, 1)
        asyncio.run(self.sio.emit('distance_update', {'distance': distance_cm}))
        
    def publish_command(self, command: str):
        msg = Twist()
        if command == 'forward': msg.linear.x = 1.0
        elif command == 'backward': msg.linear.x = -1.0
        elif command == 'left': msg.angular.z = 1.0
        elif command == 'right': msg.angular.z = -1.0
        elif command == 'stop': msg.linear.x = 0.0; msg.angular.z = 0.0
        self.cmd_vel_publisher_.publish(msg)

    def gps_callback(self, msg):
        asyncio.run(self.sio.emit('gps_update', {'lat': msg.latitude, 'lon': msg.longitude}))

    # --- تابع callback اصلاح شده برای پردازش صحیح تصویر ---
    def image_callback(self, msg):
        try:
            # تغییر کلیدی اینجاست: ما به طور مستقیم فرمت rgb8 را می‌خوانیم
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            
            # چون OpenCV به فرمت BGR نیاز دارد، یک بار رنگ‌ها را جابجا می‌کنیم
            cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            # تصویر صحیح را به فرمت JPEG فشرده می‌کنیم
            _, buffer = cv2.imencode('.jpg', cv_image_bgr)
            
            # تصویر را برای ارسال آماده می‌کنیم
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            
            # تصویر صحیح را به رابط کاربری می‌فرستیم
            asyncio.run(self.sio.emit('video_update', jpg_as_text))
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

# --- راه‌اندازی سرور ---
app = FastAPI()
sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins='*')
socket_app = socketio.ASGIApp(sio)
app.mount("/socket.io", socket_app)

# --- مسیردهی مطلق و پایدار به پوشه frontend ---
backend_dir = os.path.dirname(os.path.abspath(__file__))
frontend_dir = os.path.join(os.path.dirname(backend_dir), "frontend")
app.mount("/", StaticFiles(directory=frontend_dir, html=True), name="static")
# --------------------------------------------------

rclpy.init()
ros_node = RosBridgeNode(sio_server=sio)
ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
ros_thread.start()

@sio.event
async def connect(sid, environ): print(f"✅ Client connected: {sid}")
@sio.event
async def disconnect(sid): print(f"❌ Client disconnected: {sid}")
@sio.on('control')
async def on_control_message(sid, data):
    ros_node.publish_command(data.get('command'))