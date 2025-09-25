from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    project_base_path = '/home/amin/robot_project'
    
    bringup_pkg_scripts_path = os.path.join(project_base_path, 'ros2_ws', 'src', 'robot_bringup', 'robot_bringup')
    
    serial_bridge_script = os.path.join(bringup_pkg_scripts_path, 'serial_bridge_node.py')
    
    return LaunchDescription([
        
        ExecuteProcess(
            cmd=['python3', serial_bridge_script],
            name='serial_bridge_node',
            output='screen'
        ),
        
        Node(
            package='camera_ros',
            executable='camera_node',
            name='pi_camera',
            parameters=[
                {'pixel_format': 'YUYV'}, 
                {'image_width': 640},
                {'image_height': 480},
                {'framerate': 15.0},
            ],
            remappings=[
                ('/pi_camera/image_raw', '/video_stream')
            ]
        ),
        
        ExecuteProcess(
            cmd=[f'{project_base_path}/web_gui/venv/bin/python', '-m', 'uvicorn', 'backend.main:app', '--host', '0.0.0.0', '--port', '8000'],
            cwd=f'{project_base_path}/web_gui',
            output='screen'
        )
    ])