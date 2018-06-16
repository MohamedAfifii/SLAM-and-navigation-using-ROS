#!/bin/bash
rosrun map_server map_saver -f ~/Graduation_project/SLAM_and_navigation_using_ROS/src/rikobot_nav/maps/map map:=/rtabmap/grid_map

rosrun map_server map_server ~/Graduation_project/SLAM_and_navigation_using_ROS/src/rikobot_nav/maps/map.yaml map:=/map
