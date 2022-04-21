#!/bin/bash -e

if [ "${SIMULATION+x}" != "" ]; then
    exec ros-with-env ros2 launch rplidar_ros2 static_tf_launch.py
else
    exec ros-with-env ros2 launch rplidar_ros2 sensors_launch.py
fi
