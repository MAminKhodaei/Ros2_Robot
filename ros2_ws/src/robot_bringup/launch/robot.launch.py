# =================================================================
# ==     robot.launch.py - نسخه نهایی (با مسیرهای صحیح)          ==
# =================================================================
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    project_base_path = '/home/amin/robot_project'
    
    # مسیر صحیح به پوشه داخلی پکیج شما
    bringup_pkg_scripts_path = os.path.join(project_base_path, 'ros2_ws', 'src', 'robot_bringup', 'robot_bringup')
    
    # مسیر صحیح اسکریپت پل ارتباطی سریال
    serial_bridge_script = os.path.join(bringup_pkg_scripts_path, 'serial_bridge_node.py')
    
    return LaunchDescription([
        
        # 1. اجرای پل ارتباطی سریال (با مسیر صحیح)
        ExecuteProcess(
            cmd=['python3', serial_bridge_script],
            name='serial_bridge_node',
            output='screen'
        ),
        
        # 2. اجرای گره دوربین (با روش استاندارد Node)
        Node(
            package='camera_ros',
            executable='camera_node',
            name='pi_camera',
            parameters=[
                {'camera_name': 'pi_camera'},
                {'frame_id': 'camera_link'},
                {'pixel_format': 'rgb8'},
                {'image_width': 640},
                {'image_height': 480},
                {'framerate': 15.0},
            ],
            remappings=[
                ('/pi_camera/image_raw', '/video_stream')
            ]
        ),
        
        # 3. اجرای وب سرور (Backend)
        ExecuteProcess(
            cmd=[f'{project_base_path}/web_gui/venv/bin/python', '-m', 'uvicorn', 'backend.main:app', '--host', '0.0.0.0', '--port', '8000'],
            cwd=f'{project_base_path}/web_gui',
            output='screen'
        )
    ])