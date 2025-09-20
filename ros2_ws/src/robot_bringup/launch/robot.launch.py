# =================================================================
# ==              robot.launch.py - نسخه نهایی و قطعی             ==
# =================================================================
# این نسخه گره‌های پایتون را مستقیماً اجرا کرده و colcon را دور می‌زند

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # مسیر ریشه پروژه شما
    project_base_path = '/home/amin/robot_project'
    
    # مسیر ورک‌اسپیس ROS 2
    ros2_ws_path = os.path.join(project_base_path, 'ros2_ws')
    
    # مسیر دقیق اسکریپت‌های پایتون شما
    # توجه: این مسیر به پوشه 'src' اشاره دارد، نه 'install'
    bringup_pkg_path = os.path.join(ros2_ws_path, 'src', 'robot_bringup')
    serial_bridge_script = os.path.join(bringup_pkg_path, 'serial_bridge_node.py')
    
    return LaunchDescription([
        
        # 1. اجرای پل ارتباطی سریال (به صورت مستقیم)
        ExecuteProcess(
            cmd=['python3', serial_bridge_script],
            name='serial_bridge_node',
            output='screen'
        ),
        
        # 2. اجرای گره دوربین رسمی (این چون از قبل نصب شده، با Node اجرا می‌شود)
        # این بخش بدون تغییر است
        ExecuteProcess(
            cmd=['ros2', 'run', 'camera_ros', 'camera_node'],
            name='pi_camera',
            output='screen',
            # اضافه کردن پارامترها از طریق خط فرمان
            additional_env={'ROS_PARAMS_FILE': os.path.join(bringup_pkg_path, 'launch/camera_params.yaml')}
        ),
        
        # 3. اجرای وب سرور (Backend) (بدون تغییر)
        ExecuteProcess(
            cmd=[f'{project_base_path}/web_gui/venv/bin/python', '-m', 'uvicorn', 'backend.main:app', '--host', '0.0.0.0', '--port', '8000'],
            cwd=f'{project_base_path}/web_gui',
            output='screen'
        )
    ])