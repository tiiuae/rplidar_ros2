FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-72709dd

ARG TARGETARCH

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

# The install-<TARGETARCH>.xz file contains the install directory. This file
# has been generated during cross-compilation.
# See Dockerfile.builder in this repo.
COPY install-${TARGETARCH}.tar.gz install.tar.gz
# WORKSPACE_DIR environment variable is defined in the fog-ros-baseimage.
# The same installation directory is used by all ROS2 components.
# See: https://github.com/tiiuae/fog-ros-baseimage/blob/main/Dockerfile
RUN md5sum install.tar.gz && \
    mkdir -p $WORKSPACE_DIR && \
    tar -xzf install.tar.gz --directory $WORKSPACE_DIR
