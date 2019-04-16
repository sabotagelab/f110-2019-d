cmake_minimum_required(VERSION 2.8.3)
project(pemdas_gap_finding)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  Gap.msg
  Gaps.msg
)

##catkin_python_setup()

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS ackermann_msgs geometry_msgs roscpp rospy sensor_msgs std_msgs message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

