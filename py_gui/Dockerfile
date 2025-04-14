FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-noetic-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# Copy GUI code
WORKDIR /app
COPY . /app

# Install Python dependencies
RUN pip3 install -r requirements.txt

# Set ROS environment variables
ENV ROS_MASTER_URI=http://ros-master:11311
ENV ROS_HOSTNAME=gui

# Run the GUI
CMD ["python3", "src/main.py"]