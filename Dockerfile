# Given dynamically from CI job.
FROM --platform=${BUILDPLATFORM:-linux/amd64} ghcr.io/tiiuae/fog-ros-sdk:v3.3.0-${TARGETARCH:-amd64} AS builder

# Must be defined another time after "FROM" keyword.
ARG TARGETARCH

# SRC_DIR environment variable is defined in the fog-ros-sdk image.
# The same workspace path is used by all ROS2 components.
# See: https://github.com/tiiuae/fog-ros-baseimage/blob/main/Dockerfile.sdk_builder
COPY . $SRC_DIR/rplidar_ros2

# Tar directories so they are easier to handle when doing installation.
RUN /packaging/build_colcon_sdk.sh ${TARGETARCH:-amd64}
# Even though it is possible to tar the install directory for retrieving it later in runtime image,
# the tar extraction in arm64 emulated on arm64 is still slow. So, we copy the install directory instead

FROM ghcr.io/tiiuae/fog-ros-baseimage:v3.3.0

HEALTHCHECK --interval=5s \
	CMD fog-health check --metric=rplidar_scan_count --diff-gte=1.0 \
		--metrics-from=http://localhost:${METRICS_PORT}/metrics --only-if-nonempty=${METRICS_PORT}

# launch file checks env variables SIMULATION and DRONE_AIRFRAME
# SIMULATION is by default 0. However, it can be set to 1
# DRONE_AIRFRAME is by default "t-drone". However, it can be set to "holybro"
ENTRYPOINT [ "/entrypoint.sh" ]

COPY entrypoint.sh /entrypoint.sh

# WORKSPACE_DIR environment variable is defined in the fog-ros-baseimage.
# The same installation directory is used by all ROS2 components.
# See: https://github.com/tiiuae/fog-ros-baseimage/blob/main/Dockerfile
WORKDIR $WORKSPACE_DIR

COPY --from=builder $WORKSPACE_DIR/install install
