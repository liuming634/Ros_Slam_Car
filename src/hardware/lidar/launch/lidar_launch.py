from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    lidar_pkg = get_package_share_directory('lidar')
    lidar_params = os.path.join(lidar_pkg, 'config', 'lidar_params.yaml')

    lidar_node = Node(
        package='lidar',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        parameters=[lidar_params]
    )

    return LaunchDescription([
        lidar_node,
    ])
