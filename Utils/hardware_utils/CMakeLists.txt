
cmake_minimum_required(VERSION 2.8.3)
project(hardware_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g")
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )

set(JetsonGPIO_DIR "/usr/local/lib/cmake/JetsonGPIO")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  tf
  mavros_msgs
  swarm_msgs
  dynamic_reconfigure
  JetsonGPIO
  message_generation
)

find_package(yaml-cpp REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/PID.cfg
)

add_service_files(
  FILES
  RotateMotor.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  # LIBRARIES sensor_visualization
  CATKIN_DEPENDS roscpp swarm_msgs message_runtime roslib
  DEPENDS YAML_CPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  /usr/local/include
)

add_executable(hard_mavros_bridge 
  src/hard_mavros_bridge_core.cpp
  src/callbacks_bridge.cpp
  src/gpio_bridge.cpp
  src/params_bridge.cpp
  src/utils_bridge.cpp
  src/visual_bridge.cpp
)

target_link_libraries(hard_mavros_bridge
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  JetsonGPIO
)

add_dependencies(hard_mavros_bridge ${PROJECT_NAME}_gencfg)
add_dependencies(hard_mavros_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/mavparam.py
  scripts/external_nav_test.py
  scripts/cmd_cli.py
  scripts/lcm_cmd_cli.py
  scripts/gps_spy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)