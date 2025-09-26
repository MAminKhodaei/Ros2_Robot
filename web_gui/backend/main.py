# =================================================================
# ==        main.py - نسخه نهایی با قابلیت استریم ویدیو           ==
# =================================================================
import socketio, rclpy, threading, asyncio, base64, cv2
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Image # <-- Image را اضافه کنید
from cv_bridge import CvBridge # <-- cv_bridge را اضافه کنید
from sensor_msgs.msg import Range # <-- Add this import

class RosBridgeNode(Node):
    def __init__(self, sio_server):
        super().__init__('web_gui_ros_bridge')
        self.sio = sio_server
        self.bridge = CvBridge() # <-- ساخت آبجکت cv_bridge
        
        self.cmd_vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gps_subscriber_ = self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.distance_subscriber_ = self.create_subscription(Range, 'distance', self.distance_callback, 10)
        # ---订阅 کننده جدید برای دریافت فریم‌های ویدیو---
        self.image_subscriber_ = self.create_subscription(
            Image,
            'video_stream', # نام تاپیکی که دوربین روی آن منتشر می‌کند
            self.image_callback,
            10) # عدد 10 عمق صف است
        # ----------------------------------------------
            
        self.get_logger().info('ROS 2 Bridge Node is ready.')

    # ... (تابع publish_command و gps_callback بدون تغییر باقی می‌مانند) ...
    def distance_callback(self, msg):
        # We send the distance in centimeters for easier display
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

    # --- تابع callback جدید برای پردازش هر فریم تصویر ---
    def image_callback(self, msg):
        try:
            # تبدیل پیام ROS Image به فرمت تصویر OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # فشرده‌سازی تصویر به فرمت JPEG
            _, buffer = cv2.imencode('.jpg', cv_image)
            
            # تبدیل بایت‌های تصویر به رشته Base64 برای ارسال از طریق وب
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            
            # ارسال تصویر به رابط کاربری
            asyncio.run(self.sio.emit('video_update', jpg_as_text))
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

# --- بقیه کد بدون تغییر باقی می‌ماند ---
app = FastAPI()
sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins='*')
socket_app = socketio.ASGIApp(sio)
app.mount("/socket.io", socket_app)
app.mount("/", StaticFiles(directory="frontend", html=True), name="static")

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