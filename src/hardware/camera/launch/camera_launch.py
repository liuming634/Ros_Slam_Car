from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 通过 ament_index 获取 camera 包的共享目录（source 后稳定可用）
    pkg_share = get_package_share_directory('camera')
    params_file = os.path.join(pkg_share, 'config', 'camera_params.yaml')

    camera_node = Node(
        package='camera',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        camera_node
    ])
