cmake_minimum_required(VERSION 2.8.3)
project(pemdas_runtime_monitoring)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  vesc_msgs
  message_generation
  genmsg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime vesc_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pemdas_average_vesc src/pemdas_average_vesc.cpp)
target_link_libraries(pemdas_average_vesc ${catkin_LIBRARIES})

