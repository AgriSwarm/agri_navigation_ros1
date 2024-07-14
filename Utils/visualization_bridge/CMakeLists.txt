cmake_minimum_required(VERSION 3.0.2)
project(visualization_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  rviz_visual_tools
  visualization_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES sensor_visualization
  CATKIN_DEPENDS roscpp nav_msgs rviz_visual_tools visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sensor_visualization_node 
  src/sensor_visualization.cpp
)

add_dependencies(sensor_visualization_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(sensor_visualization_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)