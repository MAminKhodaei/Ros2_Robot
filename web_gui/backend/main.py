# =================================================================
# ==              main.py - نسخه نهایی با قابلیت WebRTC            ==
# =================================================================

# -----------------------------------------------------------------------------------
# وارد کردن کتابخانه‌های اصلی
# -----------------------------------------------------------------------------------
import socketio # کتابخانه Socket.IO برای ارتباطات دوطرفه Real-time با وب
import rclpy # کتابخانه اصلی ROS 2
import threading # برای اجرای حلقه اصلی ROS 2 (executor.spin) در یک رشته جداگانه
import asyncio # برای مدیریت توابع ناهمزمان (async/await) مورد نیاز Socket.IO و WebRTC
import base64 # برای رمزگذاری داده‌های تصویر JPEG به فرمت متنی (Base64)
import cv2 # کتابخانه OpenCV برای پردازش تصویر (رمزگذاری به JPEG)
import os # برای مدیریت مسیرهای فایل در FastAPI
import numpy # برای کار با آرایه‌های داده (در OpenCV)
import uuid # برای تولید شناسه‌های یکتا برای اتصالات WebRTC (Peer Connections)

# --- کتابخانه‌های اختصاصی ROS 2 ---
from fastapi import FastAPI # فریم‌ورک وب برای مدیریت مسیرها و WebRTC
from fastapi.staticfiles import StaticFiles # برای سرویس‌دهی فایل‌های ثابت (HTML, CSS, JS)
from rclpy.node import Node # کلاس پایه برای نود ROS
from geometry_msgs.msg import Twist # پیام فرمان‌های سرعت ربات
from sensor_msgs.msg import NavSatFix, Image, Range # پیام‌های GPS، تصویر و فاصله
from cv_bridge import CvBridge # ابزار تبدیل پیام ROS Image به آرایه OpenCV و بالعکس

# --- کتابخانه‌های WebRTC ---
from aiortc import RTCPeerConnection, RTCSessionDescription # مدیریت اتصال WebRTC
from aiortc.contrib.media import MediaStreamTrack # کلاس پایه برای مدیریت Trackهای رسانه‌ای

# --- کلاس گره اصلی ROS ---
class RosBridgeNode(Node):
    """
    نود اصلی ROS 2 که پل ارتباطی بین تاپیک‌های ROS و سرور Socket.IO است.
    """
    def __init__(self, sio_server):
        super().__init__('web_gui_ros_bridge') # تعیین نام نود
        self.sio = sio_server # ارجاع به نمونه سرور Socket.IO
        self.bridge = CvBridge() # ابزار تبدیل تصویر
        
        # Publisher برای ارسال فرمان‌های حرکتی ربات از وب به ROS
        self.cmd_vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriberها برای دریافت داده‌های سنسورها و تصویر از ROS
        self.gps_subscriber_ = self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.distance_subscriber_ = self.create_subscription(Range, 'distance', self.distance_callback, 10)
        
        # Subscriber برای دریافت فید تصویر وب‌کم از ROS.
        # توجه: منبع این فید (Publisher) در نود نهایی، درایور v4l2_camera_node است.
        self.image_subscriber_ = self.create_subscription(Image, 'video_stream', self.image_callback, 10)
        
        self.get_logger().info('ROS 2 Bridge Node is ready.')

    def distance_callback(self, msg):
        """دریافت داده فاصله (Range) و ارسال آن به تمام کلاینت‌های Socket.IO"""
        # تبدیل فاصله از متر به سانتی‌متر و گرد کردن
        # استفاده از asyncio.run برای فراخوانی متد ناهمزمان Socket.IO
        asyncio.run(self.sio.emit('distance_update', {'distance': round(msg.range * 100, 1)}))
        
    def publish_command(self, command: str):
        """دریافت فرمان کنترلی از وب و انتشار آن در تاپیک cmd_vel"""
        msg = Twist()
        # تعریف منطق تبدیل فرمان‌های متنی وب به پیام Twist
        if command == 'forward': msg.linear.x = 1.0
        elif command == 'backward': msg.linear.x = -1.0
        elif command == 'left': msg.angular.z = 1.0
        elif command == 'right': msg.angular.z = -1.0
        else: msg.linear.x = 0.0; msg.angular.z = 0.0 # توقف
        self.cmd_vel_publisher_.publish(msg) # انتشار پیام Twist

    def gps_callback(self, msg):
        """دریافت داده GPS و ارسال آن به وب"""
        asyncio.run(self.sio.emit('gps_update', {'lat': msg.latitude, 'lon': msg.longitude}))

    def image_callback(self, msg):
        """دریافت فریم تصویر از ROS، تبدیل و ارسال آن به وب از طریق Socket.IO (به عنوان فید پشتیبان)"""
        try:
            # تبدیل پیام ROS Image به آرایه NumPy (فرمت OpenCV)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # رمزگذاری تصویر (OpenCV) به فرمت JPEG برای فشرده‌سازی
            _, buffer = cv2.imencode('.jpg', cv_image)
            # تبدیل بافِر JPEG به Base64 (متنی) برای ارسال از طریق Socket.IO
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            # ارسال فید تصویر به وب
            asyncio.run(self.sio.emit('video_update', jpg_as_text))
        except Exception as e:
            self.get_logger().error(f"Error processing robot image: {e}")

# --- گره جدید برای مدیریت دوربین خارجی (WebRTC) ---
class ExternalCameraNode(Node):
    """
    این نود برای دریافت فریم‌های وب‌کمی که از طریق WebRTC به سرور می‌رسد و انتشار آن
    به عنوان یک تاپیک ROS (در صورت نیاز به پردازش در ROS) استفاده می‌شود.
    """
    def __init__(self):
        super().__init__('external_camera_publisher')
        # Publisher برای انتشار فریم‌های دریافتی از WebRTC در یک تاپیک جداگانه
        self.publisher = self.create_publisher(Image, '/external_camera/image_raw', 10)
        self.bridge = CvBridge()
        self.get_logger().info("External Camera Publisher is ready.")

    def publish_frame(self, frame):
        """تبدیل فریم OpenCV دریافتی از WebRTC و انتشار آن در ROS"""
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Could not publish external camera frame: {e}")

# --- راه‌اندازی سرور و ROS ---

# راه‌اندازی سرور FastAPI
app = FastAPI()
# راه‌اندازی سرور ناهمگام Socket.IO با CORS فعال
sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins='*')
socket_app = socketio.ASGIApp(sio)
app.mount("/socket.io", socket_app) # mount کردن Socket.IO به مسیر /socket.io

# مسیردهی فایل‌های ثابت (Frontend)
backend_dir = os.path.dirname(os.path.abspath(__file__))
frontend_dir = os.path.join(os.path.dirname(backend_dir), "frontend")
app.mount("/", StaticFiles(directory=frontend_dir, html=True), name="static") # سرویس‌دهی فایل‌های HTML/CSS/JS

# راه‌اندازی محیط ROS 2
rclpy.init()
ros_bridge_node = RosBridgeNode(sio_server=sio) # ایجاد نود اصلی
external_camera_node = ExternalCameraNode() # ایجاد نود دوربین خارجی (WebRTC)
executor = rclpy.executors.MultiThreadedExecutor() # استفاده از Executor چندرشته‌ای
executor.add_node(ros_bridge_node) # اضافه کردن نودها به Executor
executor.add_node(external_camera_node)
ros_thread = threading.Thread(target=executor.spin, daemon=True) # اجرای حلقه ROS در یک رشته پس‌زمینه
ros_thread.start()

# --- منطق جدید WebRTC ---
pcs = set() # مجموعه‌ای برای نگهداری اتصالات فعال RTCPeerConnection

class VideoTransformTrack(MediaStreamTrack):
    """
    یک واسط سفارشی برای دریافت داده از WebRTC و انتقال آن به نود ROS (ExternalCameraNode).
    """
    kind = "video"
    def __init__(self, track):
        super().__init__()
        self.track = track

    async def recv(self):
        # دریافت فریم رسانه‌ای (Media Frame) از WebRTC
        frame = await self.track.recv()
        # تبدیل فریم به آرایه NumPy در فرمت BGR24
        img = frame.to_ndarray(format="bgr24")
        # انتشار فریم دریافت شده در تاپیک ROS
        external_camera_node.publish_frame(img)
        return frame # بازگرداندن فریم به WebRTC

@app.post("/offer")
async def offer(request: dict):
    """
    Endpoint برای مدیریت تبادل SDP (Session Description Protocol)
    """
    # ایجاد یک شیء RTCSessionDescription از درخواست کلاینت
    offer = RTCSessionDescription(sdp=request["sdp"], type=request["type"])
    pc = RTCPeerConnection() # ایجاد اتصال جدید
    pc_id = f"PeerConnection({uuid.uuid4()})"
    pcs.add(pc) # اضافه کردن اتصال به مجموعه فعال
    
    @pc.on("track")
    def on_track(track):
        """رویداد هنگام دریافت Track (مثل ویدیو) از کلاینت"""
        if track.kind == "video":
            # اضافه کردن واسط سفارشی VideoTransformTrack برای پردازش فریم‌های ویدیو
            pc.addTrack(VideoTransformTrack(track))
            
    await pc.setRemoteDescription(offer) # تنظیم پیشنهاد کلاینت
    answer = await pc.createAnswer() # ایجاد پاسخ (Answer)
    await pc.setLocalDescription(answer) # تنظیم پاسخ محلی
    # بازگرداندن پاسخ به کلاینت
    return {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}

@app.on_event("shutdown")
async def on_shutdown():
    """بستن همه اتصالات WebRTC هنگام خاموش شدن سرور"""
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

# --- رویدادهای Socket.IO ---
@sio.event
async def connect(sid, environ): print(f"✅ Client connected: {sid}") # رویداد اتصال کلاینت
@sio.event
async def disconnect(sid): print(f"❌ Client disconnected: {sid}") # رویداد قطع اتصال کلاینت
@sio.on('control')
async def on_control_message(sid, data):
    """دریافت فرمان کنترلی از Socket.IO و ارسال به نود ROS"""
    ros_bridge_node.publish_command(data.get('command'))
