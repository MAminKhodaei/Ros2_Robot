# launch/robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    project_base_path = '/home/amin/robot_project' 

    return LaunchDescription([
        
        # 1. اجرای پل ارتباطی سریال ما (جایگزین micro-ros-agent)
        Node(
            package='robot_bringup',
            executable='serial_bridge_node',
            name='serial_bridge'
        ),
        
        # 2. اجرای گره دوربین رسمی (بدون تغییر)
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
        
        # 3. اجرای وب سرور (Backend) (بدون تغییر)
        ExecuteProcess(
            cmd=[f'{project_base_path}/web_gui/venv/bin/python', '-m', 'uvicorn', 'backend.main:app', '--host', '0.0.0.0', '--port', '8000'],
            cwd=f'{project_base_path}/web_gui',
            output='screen'
        )
    ])