# Base CI image for mimosa (ROS2) — pre-builds all heavy dependencies so PR CI
# only needs to compile mimosa itself (~3-5 min instead of 30+ min).
#
# Build: triggered by .github/workflows/docker.yml (manual or on Dockerfile changes)
# Registry: ghcr.io/ntnu-arl/mimosa/ci-base-ros2-<distro>:{amd64,arm64}

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive

# System and ROS2 packages required by mimosa
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgoogle-glog-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    git \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-rosbag2-cpp \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/*

# Clone dependency sources (mirrors README build procedure)
RUN mkdir -p /colcon_ws/src
WORKDIR /colcon_ws/src
RUN git clone --depth 1 https://github.com/ntnu-arl/config_utilities.git -b dev/mimosa \
 && git clone --depth 1 https://github.com/ntnu-arl/gtsam.git -b feature/imu_factor_with_gravity \
 && git clone --depth 1 https://github.com/ntnu-arl/gtsam_points.git -b minimal_updated

# Build the dependency workspace
WORKDIR /colcon_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && colcon build \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS=-Wno-error \
    -DGTSAM_POSE3_EXPMAP=ON \
    -DGTSAM_ROT3_EXPMAP=ON \
    -DGTSAM_USE_QUATERNIONS=ON \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_WITH_TBB=OFF

# Workspace is ready at /colcon_ws — CI will add mimosa to src/ and build it.
