from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    imu_pkg = get_package_share_directory('imu')
    imu_params = os.path.join(imu_pkg, 'config', 'imu_params.yaml')

    imu_node = Node(
        package='imu',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[imu_params]
    )

    return LaunchDescription([
        imu_node,
    ])
