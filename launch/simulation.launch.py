import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ============================================================
# 一键仿真启动：Gazebo + 小车模型 + Cartographer 建图
#
# 话题/TF 对应关系（全部对齐 config/vi_car/cartographer/vi_car_2d.lua）：
#   Gazebo 插件发布：
#     /scan     sensor_msgs/LaserScan  frame=laser_link（雷达插件）
#     /imu/data sensor_msgs/Imu        frame=imu_link（IMU 插件，ENU 对齐）
#     /odom     nav_msgs/Odometry      odom->base_footprint（差速插件，暂不接入 SLAM）
#     /clock    仿真时间（gzserver 提供，全体节点 use_sim_time=true）
#   Cartographer 订阅：
#     /scan（lua: num_laser_scans=1）、/imu/data（launch 里 imu->imu/data remap）
#   TF 树：
#     robot_state_publisher: base_link -> laser_link / imu_link（URDF 静态 TF）
#     cartographer:          map -> odom -> base_link（provide_odom_frame=true；
#                            差速插件 publish_odom_tf=false 避免双源冲突）
#
# 用法：
#   ros2 launch launch/simulation.launch.py            # Gazebo界面 + RViz
#   ros2 launch launch/simulation.launch.py gui:=false # 无头（无Gazebo窗口）
#   车不动起来 Cartographer 收不到有效运动数据，用键盘遥控发 /cmd_vel：
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.5
# ============================================================


def generate_launch_description():
    # 项目根目录（本文件位于 <repo>/launch/ 下）
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # 小车 URDF（Gazebo 生成用；robot_state_publisher 在 cartographer.launch.py 里）
    urdf_file = os.path.join(repo_root, 'urdf', 'vi_car.urdf')

    # ---------- 可开关的启动参数 ----------
    use_gui = LaunchConfiguration('gui', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # ---------- Gazebo：物理服务端 + 图形界面（默认空世界） ----------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': use_gui}.items(),
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
        launch_arguments={'use_sim': 'true', 'use_rviz': use_rviz}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='false=不启动Gazebo图形界面（无头仿真）'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='false=不启动RViz2'),
        gazebo,
        spawn_entity,
        cartographer,
    ])
