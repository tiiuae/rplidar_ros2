from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.substitutions import ThisLaunchFileDir
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import sys

def generate_launch_description():

    ld = LaunchDescription()

    pkg_name = "rplidar_ros2"
    pkg_share_path = get_package_share_directory(package_name=pkg_name)

    # Environment variables
    DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')

    # If the SIMULATION environment variable is set to 1, then only static tf publisher will start.
    SIMULATION = os.getenv('SIMULATION')
    simulation_mode = (SIMULATION == "1")

    DRONE_TYPE = os.getenv('DRONE_TYPE')
    is_robot_holybro_type = (DRONE_TYPE == "HOLYBRO")

    # Namespace declarations
    namespace = DRONE_DEVICE_ID

    # Frame names
    fcu_frame = DRONE_DEVICE_ID + "/fcu"         # the same definition is in static_tf_launch.py file
    rplidar_frame = DRONE_DEVICE_ID + "/rplidar" # the same definition is in static_tf_launch.py file
    garmin_frame = DRONE_DEVICE_ID + "/garmin"   # the same definition is in static_tf_launch.py file

    # simulation
    ld.add_action(
        LogInfo(msg='--- SIMULATION CONFIGURATION ---', condition=IfCondition(PythonExpression([str(simulation_mode)]))),
    ),
    # rplidar driver launch
    ld.add_action(
        Node(
            namespace = namespace,
            package = pkg_name,
            executable = 'rplidar',
            condition=IfCondition(PythonExpression(['not ', str(simulation_mode)])),
            name = 'rplidar',
            remappings=[
                # publishers
                ('topic_filtered_out', '~/scan_filtered'),
                ('topic_raw_out', '~/scan'),
            ],
            parameters = [
                pkg_share_path + '/config/params.yaml',
                {"frame_id": rplidar_frame},
            ],
            output = 'screen',
        ),
    ),


    # sensor tf launch
    ld.add_action(
        LogInfo(msg='--- HOLYBRO TYPE CONFIGURATION ---', condition=IfCondition(PythonExpression([str(is_robot_holybro_type)]))),
    ),
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/static_tf_holybro_launch.py']),
            condition=IfCondition(PythonExpression([str(is_robot_holybro_type)])),
        ),
    ),

    ld.add_action(
        LogInfo(msg='--- T-DRONE TYPE CONFIGURATION ---', condition=IfCondition(PythonExpression(['not ', str(is_robot_holybro_type)]))),
    ),
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/static_tf_tdrone_launch.py']),
            condition=IfCondition(PythonExpression(['not ', str(is_robot_holybro_type)])),
        ),
    ),

    return ld
