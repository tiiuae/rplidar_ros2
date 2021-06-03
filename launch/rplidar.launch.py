from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from os import path
import os
from launch import LaunchDescription

def generate_launch_description():
    DRONE_DEVICE_ID=os.getenv('DRONE_DEVICE_ID')
    return LaunchDescription([
        Node(
            node_name='rplidar',
            package='rplidar_ros2',
            node_executable='rplidar',
            output='screen',
            parameters=[
                path.join(get_package_share_directory('rplidar_ros2'), 'config',
                          'rplidar.yaml'),
                {'topic_name': DRONE_DEVICE_ID + '/rplidar/scan'}
            ],
        ),
    ])
