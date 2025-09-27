# =================================================================
# ==        main.py - نسخه نهایی با قابلیت WebRTC                 ==
# =================================================================
import socketio, rclpy, threading, asyncio, base64, cv2, os, numpy
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Image, Range
from cv_bridge import CvBridge
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaRelay

# --- کلاس گره اصلی ROS (بدون تغییر) ---
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
        asyncio.run(self.sio.emit('distance_update', {'distance': round(msg.range * 100, 1)}))
        
    def publish_command(self, command: str):
        msg = Twist()
        if command == 'forward': msg.linear.x = 1.0
        elif command == 'backward': msg.linear.x = -1.0
        elif command == 'left': msg.angular.z = 1.0
        elif command == 'right': msg.angular.z = -1.0
        else: msg.linear.x = 0.0; msg.angular.z = 0.0
        self.cmd_vel_publisher_.publish(msg)

    def gps_callback(self, msg):
        asyncio.run(self.sio.emit('gps_update', {'lat': msg.latitude, 'lon': msg.longitude}))

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            _, buffer = cv2.imencode('.jpg', cv_image)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            asyncio.run(self.sio.emit('video_update', jpg_as_text))
        except Exception as e:
            self.get_logger().error(f"Error processing robot image: {e}")

# --- گره جدید برای مدیریت دوربین خارجی ---
class ExternalCameraNode(Node):
    def __init__(self):
        super().__init__('external_camera_publisher')
        self.publisher = self.create_publisher(Image, '/external_camera/image_raw', 10)
        self.bridge = CvBridge()
        self.get_logger().info("External Camera Publisher is ready.")

    def publish_frame(self, frame):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Could not publish external camera frame: {e}")

# --- راه‌اندازی سرور و ROS ---
app = FastAPI()
sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins='*')
socket_app = socketio.ASGIApp(sio)
app.mount("/socket.io", socket_app)

backend_dir = os.path.dirname(os.path.abspath(__file__))
frontend_dir = os.path.join(os.path.dirname(backend_dir), "frontend")
app.mount("/", StaticFiles(directory=frontend_dir, html=True), name="static")

rclpy.init()
ros_bridge_node = RosBridgeNode(sio_server=sio)
external_camera_node = ExternalCameraNode()
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(ros_bridge_node)
executor.add_node(external_camera_node)
ros_thread = threading.Thread(target=executor.spin, daemon=True)
ros_thread.start()

# --- منطق جدید WebRTC ---
pcs = set()
relay = MediaRelay()

@app.post("/offer")
async def offer(request: dict):
    offer = RTCSessionDescription(sdp=request["sdp"], type=request["type"])
    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("track")
    async def on_track(track):
        relayed_track = relay.subscribe(track)
        
        async def frame_processor():
            while True:
                try:
                    frame = await relayed_track.recv()
                    img = frame.to_ndarray(format="bgr24")
                    external_camera_node.publish_frame(img)
                except Exception:
                    break
        
        asyncio.ensure_future(frame_processor())

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    return {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}

@app.on_event("shutdown")
async def on_shutdown():
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

# --- رویدادهای Socket.IO ---
@sio.event
async def connect(sid, environ): print(f"✅ Client connected: {sid}")
@sio.event
async def disconnect(sid): print(f"❌ Client disconnected: {sid}")
@sio.on('control')
async def on_control_message(sid, data):
    ros_bridge_node.publish_command(data.get('command'))