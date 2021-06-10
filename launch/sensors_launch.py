from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import sys


def generate_launch_description():

    ld = LaunchDescription()

    # environment variables
    DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')

    # arguments
    ld.add_action(DeclareLaunchArgument("rplidar_mode", default_value="outdoor"))
    ld.add_action(DeclareLaunchArgument("serial_port", default_value="/dev/rplidar"))

    # mode select for rplidar
    # ----------------------
    # Sensitivity: optimized for longer ranger, better sensitivity but weak environment elimination 
    # Boost: optimized for sample rate 
    # Stability: for light elimination performance, but shorter range and lower sample rate 
    rplidar_mode = PythonExpression(['"Stability" if "outdoor" == "', LaunchConfiguration("rplidar_mode"), '" else "Sensitivity"'])

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
            name = 'rplidar',
            parameters = [{
                'serial_port': LaunchConfiguration("serial_port"),
                'serial_baudrate': 256000,  # A3
                'frame_id': rplidar_frame,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': rplidar_mode,
                'topic_name': 'rplidar/scan',
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
