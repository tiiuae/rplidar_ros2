#!/bin/bash -e

# launch file checks env variable SIMULATION and DRONE_TYPE 
# SIMULATION is by default 0. However, it can be set to 1
# DRONE_TYPE is by default T-DRONE. However, it can be set to HOLYBRO
exec ros-with-env ros2 launch rplidar_ros2 sensors_launch.py
