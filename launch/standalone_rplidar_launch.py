from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from os import path, getenv
from launch import LaunchDescription

def generate_launch_description():
    drone_device_id = getenv('DRONE_DEVICE_ID')
    hitl_world_id = getenv('HITL_WORLD_ID', default=None)

    params=[
        path.join(get_package_share_directory('rplidar_ros2'), 'config',
                          'params.yaml'),
    ]

    if hitl_world_id is not None:
        params.append({"sim_world_model": hitl_world_id + '/' + drone_device_id})
        params.append({"channel_type": 'sim'})
        params.append({"scan_mode": ''})

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
            parameters=params,
            output='screen',
        ),
    ])
