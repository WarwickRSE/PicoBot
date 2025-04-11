# Use the official ROS 2 humble image as the base
FROM ros:humble

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble

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

# From here we follow https://github.com/Woolfrey/control_kuka_velocity

# We wont use mujoco here as we are not using the mujoco simulator, we can put that in an additional dockerfile

# Create workspace as per the original repo
WORKDIR /root/workspace
RUN git clone https://github.com/Woolfrey/software_robot_library.git
RUN mkdir -p /root/workspace/software_robot_library/build
WORKDIR /root/workspace/software_robot_library/build
RUN cmake .. && make && make install


WORKDIR /root/workspace/ros2_ws
RUN mkdir -p src
RUN mkdir -p build
RUN mkdir -p install
RUN mkdir -p log

WORKDIR /root/workspace/ros2_ws/src
# Clone the required packages
RUN git clone https://github.com/Woolfrey/control_kuka_velocity.git
RUN git clone https://github.com/Woolfrey/client_serial_link.git
RUN git clone https://github.com/Woolfrey/server_serial_link.git
RUN git clone https://github.com/Woolfrey/interface_serial_link.git


# Build the workspace
WORKDIR /root/workspace/ros2_ws
RUN rosdep install -y --from-paths src --ignore-src -r -y
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build
