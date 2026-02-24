# Base CI image for mimosa — pre-builds all heavy dependencies so PR CI
# only needs to compile mimosa itself (~3-5 min instead of 30+ min).
#
# Build: triggered by .github/workflows/docker.yml (manual or on Dockerfile changes)
# Registry: ghcr.io/ntnu-arl/mimosa/ci-base:{amd64,arm64}

FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive

# System packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-tools \
    libgoogle-glog-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# ROS packages required by mimosa
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-cv-bridge \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-rosbag \
    ros-noetic-tf2-ros \
    ros-noetic-visualization-msgs \
    ros-noetic-message-generation \
    ros-noetic-message-runtime \
    ros-noetic-geometry-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Clone dependency sources (mirrors README build procedure)
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src
RUN git clone --depth 1 https://github.com/ntnu-arl/config_utilities.git -b dev/mimosa \
 && git clone --depth 1 https://github.com/ntnu-arl/gtsam.git -b feature/imu_factor_with_gravity \
 && git clone --depth 1 https://github.com/ntnu-arl/gtsam_points.git -b minimal_updated

# Build the dependency workspace
WORKDIR /catkin_ws
RUN . /opt/ros/noetic/setup.sh \
 && catkin init \
 && catkin config --extend /opt/ros/noetic \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_POSE3_EXPMAP=ON \
    -DGTSAM_ROT3_EXPMAP=ON \
    -DGTSAM_USE_QUATERNIONS=ON \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_WITH_TBB=OFF \
 && catkin build

# Workspace is ready at /catkin_ws — CI will add mimosa to src/ and build it.
