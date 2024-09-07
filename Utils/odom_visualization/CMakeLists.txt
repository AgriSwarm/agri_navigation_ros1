cmake_minimum_required(VERSION 2.8.3)
project(odom_visualization)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g")
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  quadrotor_msgs
  tf
  pose_utils
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES irobot_msgs
  CATKIN_DEPENDS quadrotor_msgs pose_utils
#  DEPENDS system_lib
)

###########
## Build ##
###########
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../Utils/pose_utils/include
)

link_directories(${catkin_LIBRARY_DIRS}
                 ${CMAKE_CURRENT_SOURCE_DIR}/../Utils/pose_utils/lib)

## Declare a cpp executable
add_executable(odom_visualization src/odom_visualization.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(odom_visualization
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
   -L${CMAKE_CURRENT_SOURCE_DIR}/../Utils/pose_utils/lib -lpose_utils
)