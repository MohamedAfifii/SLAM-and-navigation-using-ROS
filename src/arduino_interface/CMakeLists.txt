cmake_minimum_required(VERSION 2.8.3)
project(arduino_interface)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)


catkin_package()



include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom_translator1 src/odom.cpp)
target_link_libraries(odom_translator1 ${catkin_LIBRARIES})

add_executable(odom_translator2 src/odom2.cpp)
target_link_libraries(odom_translator2 ${catkin_LIBRARIES})

add_executable(imu_translator src/imu.cpp)
target_link_libraries(imu_translator ${catkin_LIBRARIES})

add_executable(teleop_to_cmd src/teleop_to_cmdvel.cpp)
target_link_libraries(teleop_to_cmd ${catkin_LIBRARIES})

add_executable(pose_to_odom src/pose_to_odom.cpp)
target_link_libraries(pose_to_odom ${catkin_LIBRARIES})
