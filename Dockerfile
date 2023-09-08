FROM ghcr.io/tiiuae/fog-ros-baseimage-builder:v2.1.0 AS builder

ARG HITL_SUPPORT="OFF"

RUN apt update && apt install -y --no-install-recommends \
    fakeroot lsb-release wget gnupg

RUN echo "jee" && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN [ "${HITL_SUPPORT}" = "ON" ] \
    && apt update \
    && apt install -y --no-install-recommends \
       libgz-cmake3-dev \
       libgz-utils2-dev \
       libgz-math7-dev \
       libgz-msgs9-dev \
       libgz-transport12-dev \
       protobuf-compiler \
       libprotobuf-dev \
    && rm -rf /var/lib/apt/lists/* || :

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/


RUN [ "${HITL_SUPPORT}" = "ON" ] \
    && /main_ws/src/hitl-build-and-package-as-deb.sh \
    || /packaging/build-and-package-as-deb.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:v2.1.0

ARG HITL_SUPPORT

HEALTHCHECK --interval=5s \
	CMD fog-health check --metric=rplidar_scan_count --diff-gte=1.0 \
		--metrics-from=http://localhost:${METRICS_PORT}/metrics --only-if-nonempty=${METRICS_PORT}

# launch file checks env variables SIMULATION and DRONE_AIRFRAME
# SIMULATION is by default 0. However, it can be set to 1
# DRONE_AIRFRAME is by default "t-drone". However, it can be set to "holybro"
ENTRYPOINT /entrypoint.sh

RUN apt update && apt install -y --no-install-recommends \
    lsb-release wget gnupg \
    && rm -rf /var/lib/apt/lists/*

RUN echo "jee" && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN [ "${HITL_SUPPORT}" = "ON" ] \
    && apt update \
    && apt install -y --no-install-recommends \
       libgz-msgs9 \
       libgz-transport12 \
       libprotobuf23 \
    && rm -rf /var/lib/apt/lists/* || :

COPY entrypoint.sh /entrypoint.sh
COPY --from=builder /main_ws/ros-*-rplidar-ros2_*_amd64.deb /rplidar.deb

# need update because ROS people have a habit of removing old packages pretty fast
RUN apt update && apt install -y ros-${ROS_DISTRO}-tf2-ros \
	&& dpkg -i /rplidar.deb

