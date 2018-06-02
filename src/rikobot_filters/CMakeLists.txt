cmake_minimum_required(VERSION 2.8.3)
project(rikobot_filters)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
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

add_executable(our_filter src/our_filter.cpp)
target_link_libraries(our_filter ${catkin_LIBRARIES})
add_dependencies(our_filter rikobot_filters_generate_messages_cpp)

