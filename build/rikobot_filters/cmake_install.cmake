# Install script for directory: /home/afifi/Graduation_project/SLAM_and_navigation_using_ROS/src/rikobot_filters

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/afifi/Graduation_project/SLAM_and_navigation_using_ROS/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rikobot_filters/msg" TYPE FILE FILES "/home/afifi/Graduation_project/SLAM_and_navigation_using_ROS/src/rikobot_filters/msg/sensor_readings.msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rikobot_filters/cmake" TYPE FILE FILES "/home/afifi/Graduation_project/SLAM_and_navigation_using_ROS/build/rikobot_filters/catkin_generated/installspace/rikobot_filters-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/afifi/Graduation_project/SLAM_and_navigation_using_ROS/devel/include/rikobot_filters")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/afifi/Graduation_project/SLAM_and_navigation_using_ROS/devel/share/roseus/ros/rikobot_filters")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/afifi/Graduation_project/SLAM_and_navigation_using_ROS/devel/share/common-lisp/ros/rikobot_filters")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/afifi/Graduation_project/SLAM_and_navigation_using_ROS/devel/share/gennodejs/ros/rikobot_filters")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/afifi/Graduation_project/SLAM_and_navigation_using_ROS/devel/lib/python2.7/dist-packages/rikobot_filters")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/afifi/Graduation_project/SLAM_and_navigation_using_ROS/devel/lib/python2.7/dist-packages/rikobot_filters")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/afifi/Graduation_project/SLAM_and_navigation_using_ROS/build/rikobot_filters/catkin_generated/installspace/rikobot_filters.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rikobot_filters/cmake" TYPE FILE FILES "/home/afifi/Graduation_project/SLAM_and_navigation_using_ROS/build/rikobot_filters/catkin_generated/installspace/rikobot_filters-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rikobot_filters/cmake" TYPE FILE FILES
    "/home/afifi/Graduation_project/SLAM_and_navigation_using_ROS/build/rikobot_filters/catkin_generated/installspace/rikobot_filtersConfig.cmake"
    "/home/afifi/Graduation_project/SLAM_and_navigation_using_ROS/build/rikobot_filters/catkin_generated/installspace/rikobot_filtersConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rikobot_filters" TYPE FILE FILES "/home/afifi/Graduation_project/SLAM_and_navigation_using_ROS/src/rikobot_filters/package.xml")
endif()

