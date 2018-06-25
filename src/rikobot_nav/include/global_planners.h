
#ifndef __GLOBAL_PLANNERS__
#define __GLOBAL_PLANNERS__

#include <ros/ros.h>
#include <ros/time.h>
#include <tf_wrapper.h>

#include <my_types.h>
#include <costmap.h>
#include <geometry.h>

#include <cmath>

class GlobalPlanner
{
public:
	string algorithm;
	GridPoint src, dest;		
	double max_segment_length;
	bool publish_path;

	ros::NodeHandle nh;
	ros::Publisher path_pub = ros::Publisher(nh.advertise<nav_msgs::Path>("/my_nav/path", 3));
	ros::Publisher marker_pub = ros::Publisher(nh.advertise<visualization_msgs::Marker>("visualization_marker", 10));

	double heuristic_value(GridPoint p)
	{
		if(algorithm == "Dijkstra")	return 0;
		if(algorithm == "A_star")	return EuclideanDistance(p, dest);
	}

	void remove_redundancy(vector<GridPoint> &v, GlobalCostmap& map)
	{
		int n = v.size();
		vector<GridPoint> res;
		res.push_back(v[0]);

		int i = 0;
		while(i < n-1)
		{
			int j;
			int s = i+1, e = n-1;
			while(s <= e)
			{
				int k = (s+e)/2;

				if(lineOfSight(v[i], v[k], map))	j = k, s = k+1;
				else								e = k-1;
			}

			res.push_back(v[j]);
			i = j;
		}

		v = res;
	}

	//If the goal is reachable, return the shortest path through the path object received by refrence and return true
	//Return false otherwise
	bool solve(GlobalCostmap& map, Path& path)
	{
		vector<double> backward_cost(map.width*map.height+9, -1);
		vector<GridPoint> parent(map.width*map.height+9);
		vector<bool> expanded(map.width*map.height+9, false);

		parent[map.get_index(src)] = src;

		set<QueueNode> s;
		s.insert({heuristic_value(src),src});

		int dxs[] = {-1, 0, 1}, dys[] = {-1, 0, 1};

		while(!s.empty())
		{
			QueueNode n = *(s.begin());	s.erase(n);
			GridPoint p = n.second;
			double cost = n.first - heuristic_value(p);

			int idx = map.get_index(p);
			if(expanded[idx])	continue;
			expanded[idx] = true;
			if(p == dest)	break;
			
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
		if(!expanded[map.get_index(dest)])	return false;


		//Construct the path
		vector<GridPoint> v;
		v.push_back(dest);       
		GridPoint p = dest;
		do
		{
		    p = parent[map.get_index(p)];
		    v.push_back(p);

		}while(p != src);
		reverse(v.begin(), v.end());


		//Remove redundant points
		cout << "Path smoothing ..." << endl;
		remove_redundancy(v, map);

		vector<WorldPoint> vw;
		for(GridPoint p: v)		vw.push_back(map.grid_to_world(p));

		//Stuff the path with redundant points so that the distance between any
		//two points on the path doesn't exceed max_segment_length
		int sz = vw.size();
		for(int i = 0; i < sz-1; i++)
		{
			path.push_back(vw[i]);

			WorldPoint p = vw[i], nxt = vw[i+1];
			double distance = EuclideanDistance(p, nxt);

			int numInsert = distance/max_segment_length;

			//Use the parametric equation to insert points between p and nxt
			if(numInsert)
			{
				double dt = 1.0/numInsert;
				point u = vec(point(p.x, p.y), point(nxt.x, nxt.y));

				for(double t = dt; t < 1; t += dt)
				{
					WorldPoint newPoint = p;
					newPoint.x += t*u.real();
					newPoint.y += t*u.imag();

					path.push_back(newPoint);
				}
			}
		}
		path.push_back(vw[sz-1]);



		//Path visualization
	    visualization_msgs::Marker points, line_strip;
	    points.header.frame_id = line_strip.header.frame_id = "/map";
	    points.header.stamp = line_strip.header.stamp = ros::Time::now();
	    points.ns = line_strip.ns = "points_and_lines";
	    points.action = line_strip.action = visualization_msgs::Marker::ADD;
	    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
	    points.id = 0;
	    line_strip.id = 1;
	    points.type = visualization_msgs::Marker::POINTS;
	    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	    points.scale.x = 0.07;
	    points.scale.y = 0.07;
	    line_strip.scale.x = 0.05;
	    points.color.g = 1.0f;
	    points.color.a = 1.0;
	    line_strip.color.b = 1.0;
	    line_strip.color.a = 1.0;

	    for(WorldPoint p: path)	
	    {
			points.points.push_back(p);
			line_strip.points.push_back(p);
	    }
	    marker_pub.publish(points);
	    marker_pub.publish(line_strip);


	    //Pulbish nav_msgs::Path
		if(publish_path)
		{
			nav_msgs::Path msgOut;

			msgOut.header.frame_id = "map";
			sz = path.size();
			msgOut.poses.resize(sz);
			for(int i = 0; i < sz; i++)
			{
				msgOut.poses[i].header.frame_id = "map";
				msgOut.poses[i].pose.position = path[i];
			}

			path_pub.publish(msgOut);
		}

		return true;
	}


	/*
	Mutates the map and path variables passed by reference.
	Returns 1 if it was able to contruct a valid path to the goal and store it in
	the path variable. Otherwise returns 2.
	*/
	int plan(Goal goal, GlobalCostmap &map, Path &path, string algorithm)
	{
		//Get the global costmap
		map.get_map();

		//Get source
		TFWrapper tf_wrapper;
		WorldPoint start_point;
		try
		{
			start_point = tf_wrapper.getCurrentPose().position;
		}
		catch(tf::TransformException ex)
		{
			cout << "No transformation from /map to /base_footprint is available!" << endl;
			return 2;
		}
		src = map.world_to_grid(start_point);


		//Get destination
		WorldPoint goal_point;
		goal_point = goal.pose.position;
		if(map.outside_grid(goal_point))
		{
			cout << "Goal is outside the global map!" << endl;
			return 2;
		}
		dest = map.world_to_grid(goal_point);


		printf("You are now at: (%d, %d)\n", src.first, src.second);
		printf("The goal is at: (%d, %d)\n", dest.first, dest.second);

		
		//Algorithm availability check
		vector<string> available_algorithms = {"A_star", "Dijkstra"};
		if(find(available_algorithms.begin(), available_algorithms.end(), algorithm) == available_algorithms.end())
		{
			cout << "Please choose an available algorithm for global planning!" << endl;
			cout << "Available algorithms are:" << endl;
			for(string s: available_algorithms)	cout << s << endl;
			return 2;
		}
		this->algorithm = algorithm;

		//Get path
		ros::Time start_time = ros::Time::now();
		Costmap::thresh = map.getCostVal(src)+5;
		bool can = solve(map, path);
		if(can)
		{
			ros::Duration d = ros::Time::now()-start_time;
			double t = d.toSec();
			cout << "A global path was successfully constructed in " << t << " seconds!" << endl;
			return 1;
		}
		else
		{
			cout << "Goal unreachable!" << endl;
			return 2;
		}
	}
};

#endif