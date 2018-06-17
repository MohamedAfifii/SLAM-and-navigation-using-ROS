#ifndef __MYTYPES__
#define __MYTYPES__

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <iostream>
#include <vector>
#include <set>
#include <string>

using namespace std;

typedef nav_msgs::OccupancyGrid Grid;
typedef nav_msgs::Path Path;
typedef geometry_msgs::Point WorldPoint;
typedef geometry_msgs::Pose Pose;
typedef move_base_msgs::MoveBaseActionGoal Goal;

typedef pair<int, int> GridPoint;
typedef pair<double, GridPoint> QueueNode;

#endif