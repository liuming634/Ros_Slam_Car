"""真实硬件传感器链（仿真模式下不要引入，Gazebo 插件会直接发同名话题）。

  ydlidar_ros2_driver_node  -> /scan         frame=laser_link
  ybimu_driver -> madgwick  -> /imu/data     frame=imu_link
  camera_node               -> /camera/image_raw

帧约定与 vi_car_description/urdf/vi_car.urdf 的静态 TF 一致；
雷达/IMU 驱动自身不发布 TF，统一由 robot_state_publisher 提供。

被 cartographer.launch.py 引入。可用参数：
  use_lidar   true=启动雷达驱动
  use_imu     true=启动 IMU 驱动 + Madgwick 滤波
  use_camera  true=启动相机节点
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_lidar = LaunchConfiguration('use_lidar', default='true')
    use_imu = LaunchConfiguration('use_imu', default='true')
    use_camera = LaunchConfiguration('use_camera', default='true')

    # ---------- 参数文件（各驱动包自带，不在项目 config/ 里重复维护） ----------
    # IMU 滤波参数（imu_ros2_device 包内）
    imu_filter_params = os.path.join(
        get_package_share_directory('imu_ros2_device'),
        'config', 'imu_filter_param.yaml')
    # 雷达参数（ydlidar_ros2_driver 包内，X3 标准 115200 单通道）
    ydlidar_params = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'params', 'ydlidar_x3.yaml')
    # 相机参数（camera 包内）
    camera_params = os.path.join(
        get_package_share_directory('camera'), 'config', 'camera_params.yaml')

    # ---------- 雷达链 -> /scan ----------
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[ydlidar_params],
        condition=IfCondition(PythonExpression(["'", use_lidar, "' == 'true'"])),
    )

    # ---------- IMU 链：ybimu_driver(/imu/data_raw,/imu/mag) -> madgwick(/imu/data) ----------
    ybimu_driver_node = Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
        name='ybimu_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_imu, "' == 'true'"])),
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[imu_filter_params],
        condition=IfCondition(PythonExpression(["'", use_imu, "' == 'true'"])),
    )

    # ---------- 相机链 -> /camera/image_raw ----------
    camera_node = Node(
        package='camera',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[camera_params],
        condition=IfCondition(PythonExpression(["'", use_camera, "' == 'true'"])),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true',
                              description='true=启动 YDLidar 雷达驱动'),
        DeclareLaunchArgument('use_imu', default_value='true',
                              description='true=启动 IMU 驱动 + Madgwick 滤波'),
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='true=启动相机节点'),
        ydlidar_node,
        ybimu_driver_node,
        imu_filter_node,
        camera_node,
    ])