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

add_executable(test_complementary_filter src/test_complementary_filter.cpp)
add_library(my_lib src/my_lib.cpp)
target_link_libraries(test_complementary_filter ${catkin_LIBRARIES} my_lib)
add_dependencies(test_complementary_filter rikobot_filters_generate_messages_cpp)

add_executable(pose_to_odom src/pose_to_odom.cpp)
target_link_libraries(pose_to_odom ${catkin_LIBRARIES})
