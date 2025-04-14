cmake_minimum_required(VERSION 3.14.4)
project(picobot_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf_conversions REQUIRED)
find_package(eigen_conversions REQUIRED)
find_package(dynamic_reconfigure REQUIRED)
find_package(message_generation REQUIRED)
find_package(Eigen3 REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
 #    FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SensorBias.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages()
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

# Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/ForceZ.cfg
  # cfg/DynReconf2.cfg
)


###########
## Build ##
###########
# Sets build type to "Release" in case no build type has not been set before. This is necessary to run this controller at 1 kHz.
if(NOT CMAKE_BUILD_TYPE) 
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
  add_executable(plan_send_cartesian_commands src/plan_send_cartesian_commands.cpp)
  add_executable(eye_in_hand src/eye_in_hand.cpp)
  add_executable(eye_in_hand_raster src/eye_in_hand_raster.cpp)
  add_executable(ComputePoses src/ComputePoses.cpp)
  add_executable(get_wrench_base_sim src/get_wrench_base_sim.cpp)
  add_executable(force_controller src/force_controller.cpp)
  add_executable(wrench_transform_publisher src/WrenchTransform.cpp)
  add_executable(target_to_base src/target_to_base.cpp)
  add_executable(send_target_pose src/send_target_pose.cpp)
  add_executable(tf_force_wrt_THz src/tf_force_wrt_THz.cpp)
  ## Add cmake target dependencies of the executable
  target_include_directories(plan_send_cartesian_commands
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_include_directories(eye_in_hand
      PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_include_directories(eye_in_hand_raster
      PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_include_directories(ComputePoses
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_include_directories(get_wrench_base_sim
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_include_directories(force_controller
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_include_directories(wrench_transform_publisher
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_include_directories(target_to_base
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_include_directories(send_target_pose
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_include_directories(tf_force_wrt_THz
      PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)


  ## Specify libraries to link a library or executable target against
  target_link_libraries(plan_send_cartesian_commands PUBLIC 
  rclcpp:rclcpp
  geometry_msgs::geometry_msgs
  )
  target_link_libraries(eye_in_hand PUBLIC
  rclcpp:rclcpp
  tf2_ros::tf2_ros
  geometry_msgs::geometry_msgs
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  )
  target_link_libraries(eye_in_hand_raster PUBLIC
  rclcpp:rclcpp
  tf2_ros::tf2_ros
  geometry_msgs::geometry_msgs
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  )
  target_link_libraries(ComputePoses PUBLIC
  rclcpp:rclcpp
  sensor_msgs::sensor_msgs
  tf::tf
  geometry_msgs::geometry_msgs
  rviz_visual_tools::rviz_visual_tools
  pcl_ros::pcl
  pcl_conversions::pcl_conversions)
  target_link_libraries(get_wrench_base_sim 
  rclcpp:rclcpp
  sensor_msgs::sensor_msgs
  eigen_conversions::eigen_conversions
  geometry_msgs::geometry_msgs
  tf_conversions::tf_conversions
  )
  target_link_libraries(force_controller PUBLIC
  rclcpp:rclcpp
  tf2_ros::tf2_ros
  geometry_msgs::geometry_msgs
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  message_filters::message_filters
  dynamic_reconfigure::dynamic_reconfigure)
  target_link_libraries(wrench_transform_publisher
  rclcpp:rclcpp
  geometry_msgs::geometry_msgs
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs)
  target_link_libraries(target_to_base
  rclcpp:rclcpp
  geometry_msgs::geometry_msgs
  tf2_ros::tf2_ros
  message_filters::message_filters
  tf2_geometry_msgs::tf2_geometry_msgs
  )
  target_link_libraries(send_target_pose 
  rclcpp:rclcpp
  geometry_msgs::geometry_msgs)
  target_link_libraries(tf_force_wrt_THz 
  rclcpp:rclcpp
  geometry_msgs::geometry_msgs
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs)

#############
## Install ##
#############


ament_package()
