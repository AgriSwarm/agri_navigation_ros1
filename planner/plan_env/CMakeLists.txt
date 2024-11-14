cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
  dynamic_reconfigure
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/GridMap.cfg
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp std_msgs dynamic_reconfigure
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( plan_env 
    src/grid_map.cpp 
    # src/grid_map_bigmap.cpp
    src/raycast.cpp
    )

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)


target_link_libraries( plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )  
