
cmake_minimum_required(VERSION 2.8.3)
project(hardware_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g")
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  mavros_msgs
  swarm_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES sensor_visualization
  CATKIN_DEPENDS roscpp swarm_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hard_mavros_bridge src/hard_mavros_bridge.cpp)

target_link_libraries(hard_mavros_bridge
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/mavparam.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)