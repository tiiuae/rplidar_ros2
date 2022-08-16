import launch
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = LaunchDescription()

    # environment variables
    DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')

    #namespace declarations
    namespace = DRONE_DEVICE_ID

    # frame names
    fcu_frame = DRONE_DEVICE_ID + "/fcu"
    rplidar_frame = DRONE_DEVICE_ID + "/rplidar"
    garmin_frame = DRONE_DEVICE_ID + "/garmin"

    # node definitions
    ld.add_action(
        Node(
            namespace = namespace,
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            name= "fcu_to_rplidar_static_transform_publisher",
            arguments = ["-0.078", "0", "0.0614", "3.141592", "0", "0", fcu_frame, rplidar_frame],
            output='screen',
        ),
    )
    
    ld.add_action(
        Node(
            namespace = namespace,
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            name= "fcu_to_garmin_static_transform_publisher",
            arguments = ["-0.20", "0", "0.0139", "0", "1.5708", "0", fcu_frame, garmin_frame],
            output='screen',
        ),
    )

    return ld
