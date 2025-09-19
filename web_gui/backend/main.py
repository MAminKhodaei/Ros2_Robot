import socketio
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading

# --- ROS 2 Node Definition ---
class RosBridgeNode(Node):
    def __init__(self):
        super().__init__('web_gui_ros_bridge')
        self.cmd_vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('ROS 2 Bridge Node is ready.')

    def publish_command(self, command: str):
        msg = Twist()
        if command == 'forward':
            msg.linear.x = 1.0
        elif command == 'backward':
            msg.linear.x = -1.0
        elif command == 'left':
            msg.angular.z = 1.0 # Positive rotation
        elif command == 'right':
            msg.angular.z = -1.0 # Negative rotation
        elif command == 'stop':
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        
        self.cmd_vel_publisher_.publish(msg)
        self.get_logger().info(f'Publishing to /cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

# --- FastAPI and Socket.IO Setup ---
app = FastAPI()
sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins='*')
socket_app = socketio.ASGIApp(sio)
app.mount("/socket.io", socket_app)

# Serve the frontend files
app.mount("/", StaticFiles(directory="frontend", html=True), name="static")

# --- ROS 2 Initialization and Threading ---
rclpy.init()
ros_node = RosBridgeNode()

# Run rclpy.spin in a separate thread
# This is crucial so it doesn't block our web server
ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
ros_thread.start()


# --- Socket.IO Event Handlers ---
@sio.event
async def connect(sid, environ):
    print(f"✅ Client connected: {sid}")

@sio.event
async def disconnect(sid):
    print(f"❌ Client disconnected: {sid}")

@sio.on('control')
async def on_control_message(sid, data):
    command = data.get('command')
    print(f"🛰️ Command received from GUI: {command}")
    
    # Use the ROS node to publish the command
    ros_node.publish_command(command)