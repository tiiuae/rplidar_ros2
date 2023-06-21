# Given dynamically from CI job.
ARG TARGET_ARCHITECTURE=arm64
FROM ghcr.io/tiiuae/fog-ros-sdk:sha-5f65a86-${TARGET_ARCHITECTURE}

# Must be defined another time after "FROM" keyword.
ARG TARGET_ARCHITECTURE=arm64

# SRC_DIR environment variable is defined in the fog-ros-sdk image.
# The same workspace path is used by all ROS2 components.
# See: https://github.com/tiiuae/fog-ros-baseimage/blob/main/Dockerfile.sdk_builder
COPY . $SRC_DIR/rplidar_ros2

# Tar directories so they are easier to handle when doing installation.
RUN /packaging/build_colcon_sdk.sh ${TARGET_ARCHITECTURE}
# Split the previous command into two RUN commands so the errors coming from colcon
# are easier to identify and see.
RUN du -hs install && \
    tar -czf install.tar.gz install && \
    tar -czf log.tar.gz log && \
    tar -czf build.tar.gz build && \
    rm -rf install build log && \
    ls -ah install.tar.gz
