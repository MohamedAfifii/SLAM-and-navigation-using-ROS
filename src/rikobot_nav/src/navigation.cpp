

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
	ok &= ros::param::get("/costmap/threshold_cost", map.thresh);		//map.thresh is a static variable
	ok &= ros::param::get("/global_planner/max_segment_length", global_planner.max_segment_length);
	ok &= ros::param::get("/local_planner/goal_tolerance", local_planner.goal_tolerance);
	ok &= ros::param::get("/local_planner/frequency", local_planner.frequency );
	ok &= ros::param::get("/local_planner/vmin", local_planner.vmin);
	ok &= ros::param::get("/local_planner/vmax", local_planner.vmax);
	ok &= ros::param::get("/local_planner/dv", local_planner.dv);
	ok &= ros::param::get("/local_planner/wmin", local_planner.wmin);
	ok &= ros::param::get("/local_planner/wmax", local_planner.wmax);
	ok &= ros::param::get("/local_planner/dw", local_planner.dw);
	ok &= ros::param::get("/motor/vmin", local_planner.MotorMin);
	ok &= ros::param::get("/motor/vmax", local_planner.MotorMax);
	ok &= ros::param::get("/simulation/min_trajectory_time", local_planner.min_trajectory_time);
	ok &= ros::param::get("/simulation/sim_time", local_planner.sim_time);
	ok &= ros::param::get("/simulation/sim_time_step", local_planner.sim_time_step);
	ok &= ros::param::get("/simulation/expected_velocity", local_planner.expected_velocity);
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

