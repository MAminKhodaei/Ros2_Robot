from setuptools import setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.1',
    # تغییر اصلی اینجاست: ما به طور مستقیم نام پکیج را مشخص می‌کنیم
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amin',
    maintainer_email='amin@example.com',
    description='The main package to bring up the robot',
    license='Apache-2.0',
    # گزینه 'tests_require' حذف شد تا اخطار مزاحم نمایش داده نشود
    entry_points={
        'console_scripts': [
            'camera_node = robot_bringup.camera_node:main',
            'serial_bridge_node = robot_bringup.serial_bridge_node:main',
        ],
    },
)