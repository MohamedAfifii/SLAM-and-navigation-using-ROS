cmake_minimum_required(VERSION 2.8.3)
project(rikobot_filters)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   sensor_readings.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES filters
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

