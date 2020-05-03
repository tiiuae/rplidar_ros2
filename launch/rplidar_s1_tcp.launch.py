from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            node_name='rplidar',
            package='rplidar_ros',
            node_executable='rplidar',
            output='screen',
            parameters=[{
                'channel_type': 'tcp',
                'tcp_ip': '192.168.0.7',
                'tcp_port': 20108,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }]
        ),
    ])
