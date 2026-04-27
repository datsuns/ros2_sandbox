FROM osrf/ros:humble-desktop

# Install basic dependencies, colcon, bazel, and required ROS 2 packages
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    git \
    build-essential \
    sudo \
    python3-colcon-common-extensions \
    ros-humble-pcl-conversions \
    ros-humble-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install bazelisk as bazel
RUN wget https://github.com/bazelbuild/bazelisk/releases/download/v1.20.0/bazelisk-linux-amd64 -O /usr/local/bin/bazel \
    && chmod +x /usr/local/bin/bazel

# Set the default user
USER $USERNAME
# Source ROS 2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
