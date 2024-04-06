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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Define the executable with both source files
set(EXECUTABLE_NAME rosbot)
add_executable(${EXECUTABLE_NAME} src/brightness.cpp)

# Specify dependencies for the executable
ament_target_dependencies(${EXECUTABLE_NAME} 
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

# Include directories
target_include_directories(${EXECUTABLE_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install the executable into the package (so it can be called with ros2 run pkg ...)
install(TARGETS
  ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files into the package  (so it can be called with ros2 launch pkg ...)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()