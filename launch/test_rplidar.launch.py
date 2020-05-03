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
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
        Node(
            node_name='rplidar_client',
            package='rplidar_ros',
            node_executable='rplidar_client',
            output='screen',
        ),
    ])
