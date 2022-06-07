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

    pkg_name = "rplidar_ros2"
    pkg_share_path = get_package_share_directory(pkg_name)

    # environment variables
    DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')
    # If the SIMULATION environment variable is set to 1, then only static tf publisher will start.
    SIMULATION = os.getenv('SIMULATION')

    # arguments
    ld.add_action(launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"))

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
                pkg_share_path + '/config/rplidar_a3.yaml',
                {"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                'frame_id': rplidar_frame,
                'topic_name': 'rplidar/scan',
                'topic_name_raw': 'rplidar/scan_raw',
            }],
            output = 'screen',
            parameters=[{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},],
        ),
    ),

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/static_tf_launch.py'])
        ),
    ),

    return ld
