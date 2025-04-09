# Base ROS 2 Jazzy
FROM ros:jazzy-ros-base

# Set environment
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install build tools + rosbridge
RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

# Optional: Redis client
RUN pip3 install redis

# Setup workspace
WORKDIR /root/ros2_ws
RUN mkdir -p src

# Copy only your ROS packages
COPY ./your_main_packages ./src/your_main_packages

# Build workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# Auto-source workspace
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# WebSocket port for rosbridge
EXPOSE 9090

# Default launch: rosbridge + your launch file if needed
CMD ["/bin/bash", "-c", \
  "source /opt/ros/jazzy/setup.bash && \
   source /root/ros2_ws/install/setup.bash && \
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml"]
