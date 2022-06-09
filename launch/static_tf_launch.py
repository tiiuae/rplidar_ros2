import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = LaunchDescription()

    # Launch Arguments
    ld.add_action(launch.actions.DeclareLaunchArgument("rotate_180", default_value="false"))

    # environment variables
    DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')

    #namespace declarations
    namespace = DRONE_DEVICE_ID

    # frame names
    fcu_frame = DRONE_DEVICE_ID + "/fcu"
    rplidar_frame = DRONE_DEVICE_ID + "/rplidar"
    garmin_frame = DRONE_DEVICE_ID + "/garmin"

    # Handle rotation
    if launch.substitutions.LaunchConfiguration("rotate_180") == "true":
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
            arguments = ["0", "0", "0.09", "0", "0", rotate_180, fcu_frame, rplidar_frame],
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
