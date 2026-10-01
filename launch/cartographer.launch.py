"""Cartographer 2D 建图主入口（仿真 / 真实硬件通用）。

数据来源二选一，由 use_sim 切换：
  use_sim=true  （默认）Gazebo 传感器插件直供 /scan 与 /imu/data，跳过硬件驱动
  use_sim=false 引入 include/hardware_sensors.launch.py 拉起真实雷达/IMU/相机

两条链路的话题与坐标系完全一致（frame=laser_link / imu_link，
对齐 vi_car_description 的静态 TF 与 config/cartographer/vi_car_2d.lua）。

TF 树：
  robot_state_publisher : base_link -> laser_link / camera_link / imu_link
  cartographer          : map -> odom -> base_link（provide_odom_frame=true）

用法：
  ros2 launch launch/cartographer.launch.py                  # 仿真传感器 + SLAM
  ros2 launch launch/cartographer.launch.py use_sim:=false   # 真实硬件 + SLAM
  ros2 launch launch/cartographer.launch.py use_rviz:=false  # 关可视化
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    include_dir = os.path.join(repo_root, 'launch', 'include')

    # Cartographer 项目级配置（lua；include 的 map_builder.lua 等自动从
    # cartographer 安装目录解析，无需手动复制）
    cartographer_config_dir = os.path.join(repo_root, 'config', 'cartographer')
    configuration_basename = 'vi_car_2d.lua'

    # ---------- 可开关的启动参数 ----------
    use_sim = LaunchConfiguration('use_sim', default='true')
    # 仿真默认用 /clock 仿真时间，硬件模式维持 false
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_lidar = LaunchConfiguration('use_lidar', default='true')
    use_imu = LaunchConfiguration('use_imu', default='true')
    use_camera = LaunchConfiguration('use_camera', default='true')

    # 存图：save_map:=true 时把 pbstream 写到 assets/maps/（进程退出时落盘）
    # 再用 cartographer_pbstream_to_ros_map 转 pgm/yaml 给 AMCL / Nav2
    save_map = LaunchConfiguration('save_map', default='false')
    map_save_path = LaunchConfiguration(
        'map_save_path',
        default=os.path.join(repo_root, 'assets', 'maps', 'vi_car.pbstream'))

    # 硬件链只在 use_sim:=false 时引入（仿真下由 Gazebo 代替）
    hardware_sensors = IfCondition(PythonExpression(["'", use_sim, "' != 'true'"]))

    def _cartographer_node(context, *args, **kwargs):
        """按 save_map 开关决定是否追加 -save_state_filename。

        节点参数依赖运行时才能求值的 LaunchConfiguration，所以必须用 OpaqueFunction，
        不能在 generate_launch_description() 里直接判断。
        """
        cartographer_args = [
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename,
        ]
        if IfCondition(save_map).evaluate(context):
            path = os.path.join(
                repo_root, 'assets', 'maps')
            os.makedirs(path, exist_ok=True)
            cartographer_args += ['-save_state_filename', map_save_path.perform(context)]

        return [Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=cartographer_args,
            remappings=[
                ('imu', 'imu/data'),
            ],
        )]

    cartographer_node = OpaqueFunction(function=_cartographer_node)

    # 发布 /map 栅格地图（用命令行参数，与 turtlebot3 官方 launch 一致）
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0',
        ],
    )

    # ---------- 静态 TF（base_link -> 各传感器） ----------
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ---------- 真实硬件传感器链（仅 use_sim:=false） ----------
    hardware_chain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'hardware_sensors.launch.py')),
        launch_arguments={
            'use_lidar': use_lidar,
            'use_imu': use_imu,
            'use_camera': use_camera,
        }.items(),
        condition=hardware_sensors,
    )

    # ---------- 可视化（可选） ----------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim', default_value='true',
            description='true=Gazebo仿真链(默认，/scan与/imu/data由Gazebo发布)；false=真实硬件链'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=PythonExpression(["'true' if '", use_sim, "' == 'true' else 'false'"]),
            description='仿真模式默认true(用Gazebo /clock)，硬件模式默认false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument(
            'save_map', default_value='false',
            description='true=进程退出时把 pbstream 存到 map_save_path'),
        DeclareLaunchArgument(
            'map_save_path',
            default_value=os.path.join(repo_root, 'assets', 'maps', 'vi_car.pbstream'),
            description='pbstream 输出路径（仅 save_map:=true 时生效）'),

        cartographer_node,
        occupancy_grid_node,
        robot_state_publisher,
        hardware_chain,
        rviz_node,
    ])