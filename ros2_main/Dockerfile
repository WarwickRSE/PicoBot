
# Use the official ROS 2 humble image as the base
FROM ros:humble

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

# Update system and install development tools
RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    lsb-release \
    gnupg2 \
    build-essential \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

RUN apt-get update

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Setup workspace
WORKDIR /root/workspace
# Other packages
# RUN git clone https://github.com/Other/RequiredPackages.git

# This package
WORKDIR /root/workspace/ros2_ws
RUN mkdir -p src build install log


# Copy our ROS2 code over
COPY ./src ./src

# Build the workspace
WORKDIR /root/workspace/ros2_ws
# RUN rosdep install -y --from-paths src --ignore-src -r -y

# RUN . /opt/ros/$ROS_DISTRO/setup.bash && colcon build


