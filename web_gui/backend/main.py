# =================================================================
# ==        main.py - نسخه نهایی و صحیح برای ROS 2               ==
# =================================================================
import socketio, rclpy, threading, asyncio
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

class RosBridgeNode(Node):
    def __init__(self, sio_server):
        super().__init__('web_gui_ros_bridge')
        self.sio = sio_server
        self.cmd_vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gps_subscriber_ = self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.get_logger().info('ROS 2 Bridge Node is ready.')

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

# --- راه‌اندازی سرور ---
app = FastAPI()
sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins='*')
socket_app = socketio.ASGIApp(sio)
app.mount("/socket.io", socket_app)

# مسیر صحیح به پوشه frontend
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