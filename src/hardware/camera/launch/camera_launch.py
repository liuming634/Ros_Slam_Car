from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the path to the package share directory
    pkg_share = os.getenv('AMENT_PREFIX_PATH').split(':')[0] if 'AMENT_PREFIX_PATH' in os.environ else ''
    pkg_share = os.path.join(pkg_share, 'share', 'camera')

    # Fallback to local path if AMENT_PREFIX_PATH is not set
    if not os.path.exists(os.path.join(pkg_share, 'config', 'camera_params.yaml')):
        pkg_share = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    params_file = os.path.join(pkg_share, 'config', 'camera_params.yaml')

    # Create node
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