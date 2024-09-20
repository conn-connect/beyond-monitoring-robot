cmake_minimum_required(VERSION 3.5)

project(berry_picker_node LANGUAGES CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile commands for code completion tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find other dependencies
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(realsense2 REQUIRED)

# MMDeploy
set(MMDeploy_DIR "/home/chanwoo/mmdeploy/build/install/lib/cmake/MMDeploy")
find_package(MMDeploy REQUIRED)

# xarm (Assuming you have a CMake package for xarm)
find_package(xarm REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
  ${xarm_INCLUDE_DIRS}
  ${MMDeploy_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# Source files
file(GLOB SOURCES "src/main.cpp" "src/source/*.cpp")

# Create executable
add_executable(berry_picker_node ${SOURCES})

# Target properties
target_include_directories(berry_picker_node PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
  ${xarm_INCLUDE_DIRS}
  ${MMDeploy_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(berry_picker_node
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
  ${Eigen3_LIBRARIES}
  ${Boost_LIBRARIES}
  ${realsense2_LIBRARY}
  ${xarm_LIBRARIES}
  mmdeploy
  nvinfer
  cudnn
)

# Add ament dependencies
ament_target_dependencies(berry_picker_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Install executable
install(TARGETS berry_picker_node
  DESTINATION lib/${PROJECT_NAME})

# Optionally install other files like launch files
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/)

ament_package()
