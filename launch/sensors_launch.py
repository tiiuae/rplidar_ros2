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

    # DRONE_AIRFRAME = <t-drone | holybro | rover (same as holybro)>
    is_robot_holybro_type = False
    DRONE_AIRFRAME = os.getenv('DRONE_AIRFRAME')
    if DRONE_AIRFRAME == "holybro" or DRONE_AIRFRAME == "rover":
        is_robot_holybro_type = True
    elif DRONE_AIRFRAME == "" or DRONE_AIRFRAME == "t-drone":
        is_robot_holybro_type = False
    else:
        print('ERROR: not valid DRONE_AIRFRAME.')
        sys.exit(1)

    # By default use raw scan, only if USE_FILTERED_SCAN is set to 1 or True, use filtered 
    filtered_name = "~/scan_filtered"
    raw_name = "~/scan"

    USE_FILTERED_SCAN = os.getenv('USE_FILTERED_SCAN')
    if(USE_FILTERED_SCAN == "1" or USE_FILTERED_SCAN == "True"):
        filtered_name = "~/scan"
        raw_name = "~/scan_unfiltered"
        print('[INFO] RPLIDAR: using filtered scan.')

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
                ('topic_filtered_out', filtered_name),
                ('topic_raw_out', raw_name),
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
        LogInfo(msg='--- HOLYBRO/ROVER TYPE CONFIGURATION ---', condition=IfCondition(PythonExpression([str(is_robot_holybro_type)]))),
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
