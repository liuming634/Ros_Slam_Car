"""Nav2 自主导航主入口（仿真）。

与 simulation.launch.py 的分工（两条链路互斥运行，不要同时启动）：
  simulation.launch.py    建图模式：Gazebo + 小车 + Cartographer
                          map→odom 由 Cartographer 发布（provide_odom_frame=true）
  navigation.launch.py    导航模式：Gazebo + 小车 + map_server + AMCL + Nav2
                          odom→base_footprint 由 Gazebo 差速插件发布（vi_car_nav.urdf）
                          map→odom 由 AMCL 发布

本 launch 启动的 5 部分：
  ① Gazebo + 小车      仿真环境，使用 vi_car_nav.urdf（差异见该文件注释）
  ② robot_state_publisher  静态 TF：base_footprint→base_link→laser_link/imu_link
  ③ map_server加载 assets/maps/vi_car.yaml，发布静态 /map + map→odom
  ④ amcl               粒子滤波定位：/map + /scan + /odom → 持续发布 map→odom
  ⑤ Nav2 7 节点        planner_server / controller_server / bt_navigator /
                       behavior_server / smoother_server / waypoint_follower /
                       velocity_smoother，由 lifecycle_manager_navigation 统一激活

最终输出：/cmd_vel（geometry_msgs/msg/Twist），被 Gazebo 差速插件订阅，
        内部换算成左右轮转速（wheel_separation 0.125 / wheel_diameter 0.05）。

用法：
  ros2 launch launch/navigation.launch.py                    # 完整导航 + RViz
  ros2 launch launch/navigation.launch.py gui:=false         # 无 Gazebo 界面
  ros2 launch launch/navigation.launch.py use_rviz:=false    # 无 RViz
  ros2 launch launch/navigation.launch.py map:=/abs/path/other.yaml   # 换地图

跑通后RViz 里需要两下鼠标（见 docs/navigation.md 第 5 节）：
  1. RViz 工具栏 "2D Pose Estimate"  在地图上点出小车当前真实位置 → AMCL 定位
  2. RViz 工具栏 "Nav2 Goal"          点目标位置 → 小车自主行驶过去
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    include_dir = os.path.join(repo_root, 'launch', 'include')

    description_share = get_package_share_directory('vi_car_description')

    # ---------- 可开关的启动参数 ----------
    use_gui = LaunchConfiguration('gui', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    use_composition = LaunchConfiguration('use_composition', default='False')
    use_respawn = LaunchConfiguration('use_respawn', default='False')

    # 导航模式固定使用仿真时钟（Gazebo 的 /clock）
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 导航专用 URDF：与建图版唯一差异是差速插件 publish_odom_tf=true
    urdf_file = os.path.join(
        description_share, 'urdf', 'vi_car_nav.urdf')

    # 与建图模式同一个房间场景（雷达必须能打到障碍物，否则建不出图/定位失败）
    world = LaunchConfiguration('world', default=os.path.join(
        description_share, 'worlds', 'simulation.world'))

    # 已建好的地图（Cartographer 产出，格式与 map_server 完全兼容）
    map_yaml = LaunchConfiguration(
        'map',
        default=os.path.join(repo_root, 'assets', 'maps', 'vi_car.yaml'))

    # Nav2 参数文件（vi_car 专用：footprint / robot_base_frame / 量程已按本车改好）
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            repo_root, 'config', 'nav2', 'vi_car_nav2_params.yaml'))

    # ---------- ① Gazebo 物理服务端 + 图形界面 ----------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': use_gui, 'world': world}.items(),
    )

    # ---------- ① 小车注入 Gazebo（用导航版 URDF） ----------
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', 'vi_car_nav', '-file', urdf_file],
    )

    # ---------- ② 静态 TF：base_footprint → base_link → laser_link / imu_link ----------
    #    URDF 必须传导航版，否则静态 TF 与 Gazebo 里生成的小车不一致
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'urdf_file': urdf_file,
        }.items(),
    )

    # ---------- ③④⑤ AMCL + Nav2 7 节点（官方 launch，注入本项目参数文件） ----------
    #    ★ bringup_launch.py 内部已经依次 include 了官方两个子 launch：
    #         localization_launch.py → map_server + amcl + lifecycle_manager_localization
    #         navigation_launch.py   → planner/controller/bt_navigator/behavior/
    #                                  smoother/waypoint_follower/velocity_smoother
    #                                  + lifecycle_manager_navigation
    #      所以这里【不要】再单独启动 map_server / amcl / lifecycle_manager，
    #      否则会出现同名节点抢话题、TF 双源冲突。
    #    ★ slam:='False' 显式声明：官方默认值虽为 False，但写明意图，
    #      避免后续有人改默认值时把 slam_toolbox 拉起来和 AMCL 抢 /map。
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': 'False',
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items(),
    )

    # ---------- 可视化（可选） ----------
    #    默认用 Nav2 自带 rviz 配置（已配好 /map、/scan、/plan、2D Pose Estimate、
    #    Nav2 Goal 等显示项）。本项目 src/robot/vi_car_description/rviz/ 目前为空，
    #    如需自定义可另存一份后用 rviz_config:= 指定。
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', os.path.join(
                get_package_share_directory('nav2_bringup'),
                'rviz', 'nav2_default_view.rviz'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='false=不启动 Gazebo 图形界面（无头仿真，更省内存）'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='false=不启动 RViz2'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='false=lifecycle manager 不自动激活节点（用于分阶段调试）'),
        DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='true=把 Nav2 节点加载进同一个容器（省内存，但不好排查）'),
        DeclareLaunchArgument(
            'use_respawn', default_value='False',
            description='true=节点崩溃后自动重启'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                description_share, 'worlds', 'simulation.world'),
            description='Gazebo 世界文件绝对路径，必须是有障碍物的场景'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                repo_root, 'assets', 'maps', 'vi_car.yaml'),
            description='地图 yaml 绝对路径（Cartographer 建图产出，无需转换）'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                repo_root, 'config', 'nav2', 'vi_car_nav2_params.yaml'),
            description='Nav2 参数文件绝对路径'),

        gazebo,
        spawn_entity,
        robot_state_publisher,
        bringup,
        rviz,
    ])