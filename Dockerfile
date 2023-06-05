FROM ghcr.io/tiiuae/fog-ros-baseimage-builder:sha-72709dd AS builder

# SRC_DIR environment variable is defined in the fog-ros-baseimage-builder.
# The same workspace path is used by all ROS2 components.
# See: https://github.com/tiiuae/fog-ros-baseimage/blob/main/Dockerfile.builder
COPY . $SRC_DIR/rplidar_ros2

RUN /packaging/build_colcon.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-72709dd

RUN apt update \
    && apt install -y --no-install-recommends \
        prometheus-cpp \
		tf2-ros \
    && rm -rf /var/lib/apt/lists/*

HEALTHCHECK --interval=5s \
	CMD fog-health check --metric=rplidar_scan_count --diff-gte=1.0 \
		--metrics-from=http://localhost:${METRICS_PORT}/metrics --only-if-nonempty=${METRICS_PORT}

# launch file checks env variables SIMULATION and DRONE_AIRFRAME
# SIMULATION is by default 0. However, it can be set to 1
# DRONE_AIRFRAME is by default "t-drone". However, it can be set to "holybro"
ENTRYPOINT [ "/entrypoint.sh" ]

COPY entrypoint.sh /entrypoint.sh

# INSTALL_DIR environment variable is defined in the fog-ros-baseimage.
# The same installation directory is used by all ROS2 components.
# See: https://github.com/tiiuae/fog-ros-baseimage/blob/main/Dockerfile
COPY --from=builder $INSTALL_DIR  $INSTALL_DIR
