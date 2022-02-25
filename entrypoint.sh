#!/bin/bash -e

source /opt/ros/galactic/setup.bash

if [ "${SIMULATION+x}" != "" ]; then
    ros-with-env ros2 launch rplidar_ros2 static_tf_launch.py
else
    ros-with-env ros2 launch rplidar_ros2 sensors_launch.py
fi
