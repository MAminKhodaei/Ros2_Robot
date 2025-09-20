# launch/robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # مسیر پروژه شما روی رزبری پای
    # این مسیر را با مسیر واقعی پروژه خود جایگزین کنید
    project_base_path = '/home/amin/robot_project' 

    return LaunchDescription([
        # 1. اجرای Micro-ROS Agent
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0']
        ),
        
        # 2. اجرای گره دوربین رسمی (روش پیشنهادی)
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
            # این خط تاپیک تصویر را به نامی که در وب سرور استفاده می‌کنیم، تغییر می‌دهد
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