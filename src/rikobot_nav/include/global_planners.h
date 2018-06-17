
#ifndef __GLOBAL_PLANNERS__
#define __GLOBAL_PLANNERS__

#include <ros/ros.h>
#include <ros/time.h>
#include <tf_wrapper.h>

#include <my_types.h>
#include <costmap.h>

#include <cmath>

class global_planner
{
	//Data needed for computing the heuristic value	
	string algorithm;
	GridPoint src, dest;		

	double heuristic_value(GridPoint p)
	{
		if(algorithm == "Dijkstra")	return 0;

		if(algorithm == "A_star")
		{
			int dx = dest.first - p.first;
			int dy = dest.second - p.second;

			return hypot(abs(dx), abs(dy));
		}
	}

	//If the goal is reachable, return the shortest path through the path object received by refrence and return true
	//Return false otherwise
	bool solve(GlobalCostmap& map, GridPoint src, GridPoint dest, Path& path, string algorithm)
	{
		this->src = src, this->dest = dest;
		this->algorithm = algorithm;

		vector<double> backward_cost(map.width*map.height+9, -1);
		vector<GridPoint> parent(map.width*map.height+9);
		parent[map.get_index(src)] = src;

		set<QueueNode> s;
		s.insert({heuristic_value(src),src});

		int dxs[] = {-1, 1, 0}, dys[] = {-1, 1, 0};

		while(!s.empty())
		{
			QueueNode n = *(s.begin());	s.erase(n);
			GridPoint p = n.second;
			double cost = n.first - heuristic_value(p);

			int idx = map.get_index(p);
			if(backward_cost[idx] != -1)	continue;
			backward_cost[idx] = cost;
			
			for(int dx: dxs) for(int dy: dys)
			{
				GridPoint successor = p;
				successor.first += dx, successor.second += dy;
				if(map.isOccupied(successor))	continue;

				double successorCost = cost+hypot(abs(dx), abs(dy));
				int idx = map.get_index(successor);

				if(backward_cost[idx] == -1 || successorCost < backward_cost[idx])
				{
					parent[idx] = p;
					backward_cost[idx] = successorCost;
					s.insert({successorCost+heuristic_value(successor), successor});
				}
			}
		}


		//Goal unreachable
		if(backward_cost[dest] == -1)	return false;


		//Construct the path
		vector<GridPoint> v;
		v.push_back(dest);       
		GridPoint p = dest;
		do
		{
		    p = parent[p];
		    v.push_back(p);

		}while(p != src);
		reverse(v.begin(), v.end());

		//Fill in the path message
		int sz = v.size();
		path.poses.resize(sz);
		for(int i = 0; i < sz; i++)
		{
			GridPoint grid_point = v[i];
			WorldPoint world_point = map.grid_to_world(grid_point);
			poses[i].header.frame_id = "/map";
			poses[i].pose.position = world_point;
		}

		return true;
	}
};



//Success: return 1
//Failure: return 2
int global_plan(Goal goal, Path& path, string algorithm)
{
	//Get map
	GlobalCostmap map;


	//Get source
	TFWrapper tf_wrapper;
	try
	{
		WorldPoint start_point = tf_wrapper.getCurrentPose().position;
	}
	catch(tf::TransformException ex)
	{
		cout << "No transformation from /map to /base_footprint is available!" << endl;
		return 2;
	}
	GridPoint src = map.world_to_grid(start_point);


	//Get destination
	Point goal_point;
	goal_point.x = goal.goal.target_pose.pose.position.x;
	goal_point.y = goal.goal.target_pose.pose.position.y;
	if(map.outside_grid(goal_point))
	{
		cout << "Goal is outside the global map!" << endl;
		return 2;
	}
	GridPoint dest = map.world_to_grid(goal_point);

	
	//Algorithm availability check
	vector<string> available_algorithms = {"A_star", "Dijkstra"};
	if(find(available_algorithms.begin(), available_algorithms.end(), algorithm) == available_algorithms.end())
	{
		cout << "Please choose an available algorithm for global planning!" << endl;
		cout << "Available algorithms are:" << endl;
		for(string s: available_algorithms)	cout << s << endl;
		return 2;
	}

	//Get path
	ros::Time start_time = ros::Time::now();

	global_planner planner;
	bool can = planner.solve(map, src, dest, path, algorithm);
	if(can)
	{
		ros::Duration = ros::Time::now()-start_time;
		double t = Duration.toSec();
		cout << "A global path was successfully constructed in " << t << " seconds!" << endl;
		return 1;
	}
	else
	{
		cout << "Goal unreachable!" << endl;
		return 2;
	}
}

#endif