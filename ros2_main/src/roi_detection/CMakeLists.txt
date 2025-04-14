cmake_minimum_required(VERSION 3.5)
project(roi_detection)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Include directories
include_directories(
  include
)

# Install Python scripts
install(PROGRAMS
  scripts/grn_mrkr_cropbox.py
  scripts/weighted_avg_normals.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export package dependencies
ament_package()
