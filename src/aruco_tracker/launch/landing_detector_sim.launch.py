from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('aruco_tracker'),
        'cfg',
        'single_89_sim.json'   # <-- только конфиг схемы
    ])

    return LaunchDescription([
        Node(
            package='aruco_tracker',
            executable='landing_detector_node',
            name='landing_detector',
            output='screen',
            parameters=[{
                'config_path': config_file,
                'invert': False,
                'map_frame': 'map',
                'landing_pad_frame': 'landing_plane',
                'image_topic': '/camera/image_raw',
                'camera_info_topic': '/camera/camera_info',
                'history_size': 50  # Sliding window size for median filter
            }]
        )
    ])