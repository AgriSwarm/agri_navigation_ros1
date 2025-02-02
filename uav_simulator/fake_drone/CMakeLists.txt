cmake_minimum_required(VERSION 2.8.3)
project(poscmd_2_odom)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++14")
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  quadrotor_msgs
  swarm_msgs
  mavros_msgs
  tf
  hardware_utils
)

find_package(Eigen3 REQUIRED)
find_package(rviz_visual_tools REQUIRED)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${rviz_visual_tools_INCLUDE_DIRS}
)

add_executable (poscmd_2_odom src/poscmd_2_odom.cpp )
target_link_libraries(poscmd_2_odom
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${rviz_visual_tools_LIBRARIES}
  )

