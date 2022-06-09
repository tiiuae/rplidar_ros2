import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
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

    # Launch arguments
    ld.add_action(launch.actions.DeclareLaunchArgument("mode", default_value="outdoor"))
    ld.add_action(launch.actions.DeclareLaunchArgument("rotate_180", default_value="false"))

    # Environment variables
    DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')
    # If the SIMULATION environment variable is set to 1, then only static tf publisher will start.
    SIMULATION = os.getenv('SIMULATION')
    simulation_mode = (SIMULATION == "1")

    # Namespace declarations
    namespace = DRONE_DEVICE_ID

    # Frame names
    fcu_frame = DRONE_DEVICE_ID + "/fcu"         # the same definition is in static_tf_launch.py file
    rplidar_frame = DRONE_DEVICE_ID + "/rplidar" # the same definition is in static_tf_launch.py file
    garmin_frame = DRONE_DEVICE_ID + "/garmin"   # the same definition is in static_tf_launch.py file

    # Config file path
    if launch.substitutions.LaunchConfiguration("mode")=="outdoor":
        config = os.path.join(get_package_share_directory('rplidar_ros2'),'config','rplidar_a3_outdoor.yaml')
    else:
        config = os.path.join(get_package_share_directory('rplidar_ros2'),'config','rplidar_a3_indoor.yaml')
        
    rotate_180 = launch.substitutions.LaunchConfiguration("rotate_180")

    ld.add_action(
        Node(
            namespace = namespace,
            package = 'rplidar_ros2',
            executable = 'rplidar',
            condition=IfCondition(PythonExpression(['not ', str(simulation_mode)])),
            name = 'rplidar',
            parameters = [config, {'frame_id': rplidar_frame}],
            output = 'screen',
        ),
    ),

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/static_tf_launch.py']),
            launch_arguments = {'rotate_180': rotate_180}.items()
        ),
    ),

    return ld
