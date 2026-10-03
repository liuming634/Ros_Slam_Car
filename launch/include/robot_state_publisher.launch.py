"""静态 TF 发布（base_footprint -> base_link -> laser_link / imu_link）。

URDF 从 vi_car_description 包读取（全项目坐标系的唯一真源），
经 robot_state_publisher 发布为 /tf_static。

被 cartographer.launch.py 与 navigation.launch.py 引入。可用参数：
  use_sim_time  true=Gazebo 仿真时钟（/clock），false=真实硬件
  urdf_file     URDF 绝对路径，默认建图版vi_car.urdf；
                导航模式需传入 vi_car_nav.urdf（odom TF 归属不同）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 注意：这里不能用 LaunchConfiguration 直接 open()。
    # LaunchConfiguration 要到运行时才被替换成字符串，generate_launch_description()
    # 执行时它还是 substitution 对象（open() 会报 "expected str, bytes or
    # os.PathLike object, not LaunchConfiguration"）。所以必须用 OpaqueFunction
    # 延迟到运行时求值。
    urdf_file = LaunchConfiguration(
        'urdf_file',
        default=os.path.join(
            get_package_share_directory('vi_car_description'),
            'urdf', 'vi_car.urdf'))

    def _spawn_rsp(context, *args, **kwargs):
        with open(urdf_file.perform(context), 'r') as f:
            robot_desc = f.read()

        return [Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc,
                 'use_sim_time': use_sim_time},
            ],
        )]

    robot_state_publisher = OpaqueFunction(function=_spawn_rsp)

    return LaunchDescription([robot_state_publisher])