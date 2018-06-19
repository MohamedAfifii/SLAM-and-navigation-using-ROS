

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <my_types.h>
#include <global_planners.h>
#include <local_planners.h>

ros::Publisher *pubPtr;

void call_back(const Goal& goal_point) 
{
	cout << endl << "New goal received!" << endl;
	int status = 0;
	while(!status)
	{
		Path path;
		status = global_plan(goal_point, path, "Dijkstra");

		if(status == 1)
		{
			pubPtr->publish(path);
			status = execute_path(path);
		}	
	}
}


int main(int argc , char** argv) 
{
	ros::init (argc , argv , "my_nav");
	ros::NodeHandle nh;

	pubPtr = new ros::Publisher(nh.advertise<nav_msgs::Path>("/my_nav/path", 3));
	ros::Subscriber sub = nh.subscribe("/move_base_simple/goal" , 3 ,&call_back);
	ros::spin();
}

