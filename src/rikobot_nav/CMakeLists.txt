cmake_minimum_required(VERSION 2.8.3)
project(rikobot_nav)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  move_base
  move_base_msgs
  tf
  visualization_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES messages_tutorial2
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_compile_options(-std=c++11)

add_executable(my_nav src/navigation.cpp)
target_link_libraries(my_nav ${catkin_LIBRARIES})
