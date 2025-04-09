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

# Create workspace
WORKDIR /root/ros2_ws
RUN mkdir -p src

# Clone the KUKA ROS 2 drivers, using a fork which has the cartesian and impedance control in it keep an eye on state of https://github.com/kroshu/kuka_drivers/pull/192
RUN git clone https://github.com/kroshu/kuka_drivers.git src/kuka_drivers
WORKDIR src/kuka_drivers 
RUN git fetch origin feature/fri_cart_imp_control
RUN git checkout feature/fri_cart_imp_control
WORKDIR /root/ros2_ws/src
RUN vcs import < kuka_drivers/upstream.repos

WORKDIR /root/ros2_ws
RUN rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y


# Build and source the workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build"



# Source the workspace on container start
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Default command
CMD ["bash"]
