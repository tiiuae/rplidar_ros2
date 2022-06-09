import launch
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = LaunchDescription()

    # environment variables
    DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')
    ROTATE_180 = os.getenv('ROTATE_180')

    #namespace declarations
    namespace = DRONE_DEVICE_ID

    # frame names
    fcu_frame = DRONE_DEVICE_ID + "/fcu"
    rplidar_frame = DRONE_DEVICE_ID + "/rplidar"
    garmin_frame = DRONE_DEVICE_ID + "/garmin"

    # Handle RPLidar rotation 
    if ROTATE_180 == "1":
        rotate_180="3.14"
    else:
        rotate_180="0"

    # node definitions
    ld.add_action(
        Node(
            namespace = namespace,
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            name= "fcu_to_rplidar_static_transform_publisher",
            arguments = ["0", "0", "0.09", rotate_180, "0", "0", fcu_frame, rplidar_frame],
            output='screen',
        ),
    )
    
    ld.add_action(
        Node(
            namespace = namespace,
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            name= "fcu_to_garmin_static_transform_publisher",
            arguments = ["-0.007", "-0.05", "-0.036", "0", "0", "0", fcu_frame, garmin_frame],
            output='screen',
        ),
    )

    return ld
