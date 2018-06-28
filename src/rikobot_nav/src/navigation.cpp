

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <my_types.h>
#include <global_planners.h>
#include <local_planners.h>


void call_back(const Goal& goal_point) 
{
	cout << endl << "New goal received!" << endl;

	GlobalCostmap map;
	GlobalPlanner global_planner;
	LocalPlanner local_planner;

	//Read parameters from the parameter server
	bool ok = true;
	ok &= ros::param::get("/global_planner/threshold_cost", global_planner.thresh);	
	ok &= ros::param::get("/global_planner/max_segment_length", global_planner.max_segment_length);
	ok &= ros::param::get("/global_planner/publish_path", global_planner.publish_path);
	ok &= ros::param::get("/local_planner/threshold_cost", local_planner.thresh);	
	ok &= ros::param::get("/local_planner/goal_tolerance", local_planner.goal_tolerance);
	ok &= ros::param::get("/local_planner/frequency", local_planner.frequency );
	ok &= ros::param::get("/local_planner/no_rot_angle", local_planner.no_rot_angle);
	ok &= ros::param::get("/motor/vmin", local_planner.MotorMin);
	ok &= ros::param::get("/motor/vmax", local_planner.MotorMax);
	ok &= ros::param::get("/simulation/angle_tolerance", local_planner.angle_tolerance);

	if(!ok)
	{
		cout << "One or more parameters are not ready on the parameter server" << endl;
		cout << "Aborting ..." << endl;
		return;
	}	

	int status = 0;
	while(!status)
	{
		Path path;
		status = global_planner.plan(goal_point, map, path, "Dijkstra");

		if(status == 1)
		{
			//pubPtr->publish(path);
			status = local_planner.execute_path(map, path);
		}	
	}
}


int main(int argc , char** argv) 
{
	ros::init (argc , argv , "my_nav");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/move_base_simple/goal" , 3 ,&call_back);
	ros::spin();
}

