#ifndef __MYTYPES__
#define __MYTYPES__

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <cstdio>
#include <vector>
#include <set>
#include <string>
#include <complex>
#include <cmath>

using namespace std;

typedef nav_msgs::OccupancyGrid Grid;
typedef nav_msgs::Path Path;
typedef geometry_msgs::Point WorldPoint;
typedef geometry_msgs::Pose Pose;
typedef geometry_msgs::PoseStamped Goal;
typedef pair<int, int> GridPoint;

typedef pair<double, GridPoint> QueueNode;
typedef pair<double, double> Command;

#endif