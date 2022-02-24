#!/bin/bash

source /opt/ros/galactic/setup.bash

if [ ${SIMULATION+x} != "" ]; then
    ros2 launch rplidar_ros2 static_tf_launch.py
else
    ros2 launch rplidar_ros2 sensors_launch.py
fi
