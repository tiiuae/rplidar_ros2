from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.substitutions import ThisLaunchFileDir
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import sys


def generate_launch_description():

    ld = LaunchDescription()

    # environment variables
    DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')
    # If the SIMULATION environment variable is set to 1, then only static tf publisher will start.
    SIMULATION = os.getenv('SIMULATION')

    simulation_mode = (SIMULATION == "1")
    #namespace declarations
    namespace = DRONE_DEVICE_ID

    # frame names
    fcu_frame = DRONE_DEVICE_ID + "/fcu"         # the same definition is in static_tf_launch.py file
    rplidar_frame = DRONE_DEVICE_ID + "/rplidar" # the same definition is in static_tf_launch.py file
    garmin_frame = DRONE_DEVICE_ID + "/garmin"   # the same definition is in static_tf_launch.py file

    ld.add_action(
        Node(
            namespace = namespace,
            package = 'rplidar_ros2',
            executable = 'rplidar',
            condition=IfCondition(PythonExpression(['not ', str(simulation_mode)])),
            name = 'rplidar',
            parameters = [{
                'frame_id': rplidar_frame,
                'topic_name': 'rplidar/scan',
                'topic_name_raw': 'rplidar/scan_raw',
                'channel_type': "serial",
                'tcp_ip': "192.168.0.7",
                'tcp_port': 20108,
                'serial_baudrate': 256000,
                'serial_port': "/dev/rplidar",

                'inverted': False,
                'angle_compensate': True,
                'raw_enabled': True, # Enable/Disable raw scan publish

                # Sensitivity: optimized for longer ranger, better sensitivity but weak environment elimination - indoor environment
                # Boost: optimized for sample rate
                # Stability: for light elimination performance, but shorter range and lower sample rate - outdoor environment
                'scan_mode': "Stability",

                'filter.enabled': True,
                'filter.min_range': 0.3,
                'filter.check_distance': 10.0,
                'filter.scan_search_area': 10,
                'filter.minimal_number_of_close_samples': 4,
                'filter.minimal_distance_for_acceptance_samples': 0.5,
            }],
            output = 'screen',
        ),
    ),

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/static_tf_launch.py'])
        ),
    ),

    return ld
