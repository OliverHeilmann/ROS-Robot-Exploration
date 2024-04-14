cmake_minimum_required(VERSION 3.8)
project(rosbot)

# Set C++ standard to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Define the executable for brightness with dependencies
set(EXECUTABLE_NAME_1 brightness)
add_executable(${EXECUTABLE_NAME_1} src/brightness.cpp)
ament_target_dependencies(${EXECUTABLE_NAME_1} 
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

# Define the executable for tracker with dependencies
set(EXECUTABLE_NAME_2 tracker)
add_executable(${EXECUTABLE_NAME_2} src/tracker.cpp)
ament_target_dependencies(${EXECUTABLE_NAME_2}
  rclcpp
  OpenCV
  cv_bridge
  geometry_msgs
  sensor_msgs
  std_msgs
)

set(EXECUTABLE_NAME_3 tf_broadcaster)
add_executable(${EXECUTABLE_NAME_3} src/tf_broadcaster.cpp)
ament_target_dependencies(${EXECUTABLE_NAME_3}
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

set(EXECUTABLE_NAME_4 tf_listener)
add_executable(${EXECUTABLE_NAME_4} src/tf_listener.cpp)
ament_target_dependencies(${EXECUTABLE_NAME_4}
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

# Install the executable into the package (so it can be called with ros2 run pkg ...)
install(TARGETS
  ${EXECUTABLE_NAME_1}
  ${EXECUTABLE_NAME_2}
  ${EXECUTABLE_NAME_3}
  ${EXECUTABLE_NAME_4}
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files into the package  (so it can be called with ros2 launch pkg ...)
install(DIRECTORY
  config
  launch
  saved_images
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()