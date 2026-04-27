FROM osrf/ros:jazzy-desktop

# Install basic dependencies, colcon, bazel, and required ROS 2 packages
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    git \
    build-essential \
    python3-colcon-common-extensions \
    ros-jazzy-pcl-conversions \
    ros-jazzy-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

# Install bazelisk as bazel
RUN wget https://github.com/bazelbuild/bazelisk/releases/download/v1.20.0/bazelisk-linux-amd64 -O /usr/local/bin/bazel \
    && chmod +x /usr/local/bin/bazel

# Source ROS 2 setup in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
