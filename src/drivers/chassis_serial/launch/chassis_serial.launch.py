"""底盘串口桥启动（实物底盘用，仿真不需要）。

  ros2 launch chassis_serial chassis_serial.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('chassis_serial'), 'config', 'chassis_serial.yaml')

    return LaunchDescription([
        Node(
            package='chassis_serial',
            executable='chassis_serial_node',
            name='chassis_serial_node',
            output='screen',
            parameters=[config],
        ),
    ])
