from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # 项目根目录（本文件位于 <repo>/launch/ 下）
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # Cartographer 项目级配置（lua，include 的 map_builder.lua 等自动从
    # cartographer 安装目录解析，无需手动复制）
    cartographer_config_dir = os.path.join(repo_root, 'config', 'vi_car', 'cartographer')
    configuration_basename = 'vi_car_2d.lua'

    # 小车 URDF（静态 TF：base_link -> laser_link / camera_link / imu_link）
    urdf_file = os.path.join(repo_root, 'urdf', 'vi_car.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # 相机参数（直接从源码树读取，避免依赖 camera 包是否已安装；
    # camera_node 可执行文件仍来自编译后的 camera 包，用 use_camera:=false 可跳过）
    camera_params = os.path.join(repo_root, 'src', 'hardware', 'camera', 'config', 'camera_params.yaml')

    # ---------- 可开关的启动参数 ----------
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_lidar = LaunchConfiguration('use_lidar', default='true')
    use_camera = LaunchConfiguration('use_camera', default='true')
    use_imu = LaunchConfiguration('use_imu', default='true')

    # 硬件 IMU 滤波参数（imu_ros2_device 包内配置）
    from ament_index_python.packages import get_package_share_directory
    imu_filter_params = os.path.join(
        get_package_share_directory('imu_ros2_device'), 'config', 'imu_filter_param.yaml')

    # 硬件雷达参数（ydlidar_ros2_driver 包内配置，X3 标准 115200 单通道）
    ydlidar_params = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'), 'params', 'ydlidar_x3.yaml')

    # ---------- IMU 链：ybimu_driver(/imu/data_raw,/imu/mag) -> madgwick(/imu/data) ----------
    ybimu_driver_node = Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
        name='ybimu_node',
        output='screen',
        condition=IfCondition(use_imu),
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[imu_filter_params],
        condition=IfCondition(use_imu),
    )

    # ---------- 雷达链：ydlidar_ros2_driver_node -> /scan ----------
    # frame_id=laser_link 与 urdf/vi_car.urdf 的 base_link->laser_link 一致，
    # 驱动自身不发布 TF，由 robot_state_publisher 统一提供（与例程注释掉的 tf2 一致）
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[ydlidar_params],
        condition=IfCondition(use_lidar),
    )

    # ---------- Cartographer SLAM 主节点 ----------
    # 订阅 /scan（雷达）与 /imu/data（IMU，remap），数据由真实硬件链提供
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename,
        ],
        remappings=[
            ('imu', 'imu/data'),
        ]
    )

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
        ]
    )

    camera_node = Node(
        package='camera',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[camera_params],
        condition=IfCondition(use_camera),
    )

    # ---------- TF：静态坐标变换（base_link -> 各传感器） ----------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}],
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
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),

        cartographer_node,
        occupancy_grid_node,
        ydlidar_node,
        ybimu_driver_node,
        imu_filter_node,
        camera_node,
        robot_state_publisher,
        rviz_node,
    ])
