from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from os import path
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            name='rplidar',
            package='rplidar_ros2',
            executable='rplidar',
            remappings=[
                # publishers
                ('topic_filtered_out', '~/scan_filtered'),
                ('topic_raw_out', '~/scan'),
            ],
            parameters=[
                path.join(get_package_share_directory('rplidar_ros2'), 'config',
                          'params.yaml'),
            ],
            output='screen',
        ),
    ])
