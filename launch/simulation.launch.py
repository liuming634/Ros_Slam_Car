"""Gazebo 仿真场景：Gazebo + 小车模型 + Cartographer 建图。

话题/TF 对齐关系（全部对齐 config/cartographer/vi_car_2d.lua）：
  Gazebo 插件发布：
    /scan     sensor_msgs/LaserScan  frame=laser_link（雷达插件）
    /imu/data sensor_msgs/Imu        frame=imu_link（IMU 插件，ENU 对齐）
    /odom     nav_msgs/Odometry      odom->base_footprint（差速插件，暂不接入 SLAM）
    /clock    仿真时间（全体节点 use_sim_time=true）
  TF 树：
    robot_state_publisher : base_link -> laser_link / imu_link（URDF 静态 TF）
    cartographer          : map -> odom -> base_link
                           （差速插件 publish_odom_tf=false，避免双源冲突）

关于 world（重要）：
  雷达是**水平 2D 扫描**，必须打到障碍物才会有有效回波。若用 Gazebo 默认空世界，
  所有 ranges 都是 inf → Cartographer 收到的点云为空，会被静默丢弃，
  表现为一直刷 "Queue waiting for data: (0, scan)" 且不产地图。
  因此默认加载 vi_car_description/worlds/simulation.world（内含 simulation_test 房间）。

用法：
  ros2 launch launch/simulation.launch.py                    # Gazebo 界面 + RViz + 房间场景
  ros2 launch launch/simulation.launch.py gui:=false         # 无头仿真
  ros2 launch launch/simulation.launch.py world:=<绝对路径.world>   # 换场景

车不动起来 Cartographer 收不到有效运动数据，用键盘遥控发 /cmd_vel：
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.5
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    description_share = get_package_share_directory('vi_car_description')

    # 小车 URDF（Gazebo 生成用；robot_state_publisher 在 cartographer.launch.py 里）
    urdf_file = os.path.join(description_share, 'urdf', 'vi_car.urdf')

    # 仿真世界：默认带房间的 simulation.world（雷达需要障碍物才能建图）
    world = LaunchConfiguration('world', default=os.path.join(
        description_share, 'worlds', 'simulation.world'))

    use_gui = LaunchConfiguration('gui', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    save_map = LaunchConfiguration('save_map', default='false')

    # ---------- Gazebo：物理服务端 + 图形界面 ----------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': use_gui, 'world': world}.items(),
    )

    # ---------- 把小车 URDF 生成进 Gazebo（spawn_entity 自带等服务逻辑） ----------
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', 'vi_car', '-file', urdf_file],
    )

    # ---------- Cartographer 建图链（use_sim:=true 走 Gazebo 话题，
    #            含 robot_state_publisher / occupancy_grid / RViz） ----------
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(repo_root, 'launch', 'cartographer.launch.py')),
        launch_arguments={
            'use_sim': 'true',
            'use_rviz': use_rviz,
            'save_map': save_map,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='false=不启动Gazebo图形界面（无头仿真）'),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(
                description_share, 'worlds', 'simulation.world'),
            description='Gazebo 世界文件绝对路径。必须是雷达能打到障碍物的场景，'
                        '空世界会导致 Cartographer 收不到有效点云'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='false=不启动RViz2'),
        DeclareLaunchArgument(
            'save_map', default_value='false',
            description='true=退出时把建好的地图存成 pbstream 到 assets/maps/'),
        gazebo,
        spawn_entity,
        cartographer,
    ])