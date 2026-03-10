# Base CI image for mimosa (ROS2) — pre-builds all heavy dependencies so PR CI
# only needs to compile mimosa itself (~3-5 min instead of 30+ min).
#
# Build: triggered by .github/workflows/docker.yml (manual or on Dockerfile changes)
# Registry: ghcr.io/ntnu-arl/mimosa/ci-base-ros2:{amd64,arm64}

FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive

# System packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgoogle-glog-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# ROS2 packages required by mimosa
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-cv-bridge \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-rosbag2-cpp \
    ros-jazzy-tf2-ros \
    ros-jazzy-visualization-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/*

# Clone dependency sources (mirrors README build procedure)
RUN mkdir -p /colcon_ws/src
WORKDIR /colcon_ws/src
RUN git clone --depth 1 https://github.com/ntnu-arl/config_utilities.git -b dev/mimosa \
 && git clone --depth 1 https://github.com/ntnu-arl/gtsam.git -b feature/imu_factor_with_gravity \
 && git clone --depth 1 https://github.com/ntnu-arl/gtsam_points.git -b minimal_updated

# Build the dependency workspace
WORKDIR /colcon_ws
RUN . /opt/ros/jazzy/setup.sh \
 && colcon build \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_POSE3_EXPMAP=ON \
    -DGTSAM_ROT3_EXPMAP=ON \
    -DGTSAM_USE_QUATERNIONS=ON \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_WITH_TBB=OFF

# Workspace is ready at /colcon_ws — CI will add mimosa to src/ and build it.
