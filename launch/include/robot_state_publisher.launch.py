"""静态 TF 发布（base_link -> laser_link / camera_link / imu_link）。

URDF 从 vi_car_description 包读取（全项目坐标系的唯一真源），
经 robot_state_publisher 发布为 /tf_static。

被 cartographer.launch.py 引入。可用参数：
  use_sim_time  true=Gazebo 仿真时钟（/clock），false=真实硬件
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file = os.path.join(
        get_package_share_directory('vi_car_description'), 'urdf', 'vi_car.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc, 'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([robot_state_publisher])