FROM ghcr.io/tiiuae/fog-ros-baseimage:builder-3dcb78d AS builder

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/
RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-3dcb78d

# launch file checks env variables SIMULATION and DRONE_AIRFRAME
# SIMULATION is by default 0. However, it can be set to 1
# DRONE_AIRFRAME is by default "t-drone". However, it can be set to "holybro"
ENTRYPOINT exec ros-with-env ros2 launch rplidar_ros2 sensors_launch.py

COPY --from=builder /main_ws/ros-*-rplidar-ros2_*_amd64.deb /rplidar.deb

# need update because ROS people have a habit of removing old packages pretty fast
RUN apt update && apt install -y ros-${ROS_DISTRO}-tf2-ros \
	&& dpkg -i /rplidar.deb && rm /rplidar.deb

