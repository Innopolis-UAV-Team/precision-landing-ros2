"""Simplified launch file for precision landing node."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for precision landing."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('precision_landing_ros2')
    config_file = os.path.join(pkg_dir, 'config', 'precision_landing_params.yaml')
    
    # Precision lander node
    precision_lander_node = Node(
        package='precision_landing_ros2',
        executable='precision_lander_node',
        name='precision_lander_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        precision_lander_node
    ])