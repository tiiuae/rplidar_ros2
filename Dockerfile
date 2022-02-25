# Use one base image for all the components.
# First tests can be done using debian packages of release 6.0.1.
FROM ghcr.io/tiiuae/fog-ros-baseimage

RUN apt install -y \
        ros-${ROS_DISTRO}-rplidar-ros2=1.10.0-20~git20220118.e7045d1

COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

USER runner

ENTRYPOINT /entrypoint.sh
