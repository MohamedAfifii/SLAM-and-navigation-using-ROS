#ifndef __COSTMAP__
#define __COSTMAP__

#include <ros/ros.h>
#include <my_types.h>
#include <tf_wrapper.h>


class Costmap
{
public:
	Grid map;
	int width, height;
	double resolution;
	WorldPoint origin;
	bool flag = false;

	//int thresh = 20;
	static int thresh;

	ros::NodeHandle nh;
	ros::Subscriber sub;
	string costmap_topic;

	//Update the local version of the map when receiving a message on the costmap_topic
	void call_back(const Grid& map)
	{
		this->map = map;

		width = map.info.width;
		height = map.info.height;
		resolution = map.info.resolution;
		origin = map.info.origin.position;

		flag = true;
	}

	//Subsrcribe the to costmap_topic
	//The node will keep subscribed to that topic until the costmap object goes out of scope
	void subscribe()
	{
		sub = nh.subscribe(costmap_topic, 2, &Costmap::call_back, this);
	}

	//Temporarily subscribe to the costmap_topic, receive the latest version of the map,
	//then the subscriber object will go out of scope.
	void get_map()
	{
		cout << "Waiting for the costmap ..." << endl;

		flag = false;
		ros::Subscriber temp_sub = nh.subscribe(costmap_topic, 2, &Costmap::call_back, this);
		while(!flag){ros::spinOnce();}

		cout << "Costmap received!" << endl;
	}

	bool outside_grid(WorldPoint p)
	{
		if(p.x < origin.x || p.y < origin.y)	return true;
		if(p.x > origin.x+width*resolution || p.y > origin.y+height*resolution)	return true;

		return false;
	}

	bool outside_grid(GridPoint p)
	{
		int x = p.first, y = p.second;
		return (x < 0 || y < 0 || x >= width || y >= height);
	}


	GridPoint world_to_grid(WorldPoint world_point)
	{
		if(outside_grid(world_point))	return {-1,-1};

		else
		{
			int x = (world_point.x - origin.x)/resolution;
			int y = (world_point.y - origin.y)/resolution;	
			return {x,y};
		}
	}


	WorldPoint grid_to_world(GridPoint grid_point)
	{
		WorldPoint world_point;

		world_point.x = origin.x + grid_point.first*resolution;
		world_point.y = origin.y + grid_point.second*resolution;
		world_point.z = 0;

		return world_point;
	}

	int get_index(GridPoint grid_point)
	{
		return grid_point.second*width + grid_point.first;
	}

	GridPoint get_grid_point(int index)
	{
		int y = index/width;
		int x = index%width;

		return {x,y};
	}

	int getCostVal(GridPoint grid_point)
	{
		return map.data[get_index(grid_point)];
	}
	
	int getCostVal(WorldPoint world_point)
	{
		GridPoint grid_point = world_to_grid(world_point);
		return getCostVal(grid_point);
	}

	//Assumes the grid_point lies within the grid
	bool isFree(GridPoint grid_point)
	{
		int val = getCostVal(grid_point);
		return (val != -1 && val < thresh);
	}

	//Assumes the grid_point lies within the grid
	bool isOccupied(GridPoint grid_point)
	{
		return !isFree(grid_point);
	}

	//A point outside the margins of the grid will be considered as occupied
	bool isFree(WorldPoint world_point)
	{
		if(outside_grid(world_point))	return false;

		GridPoint grid_point = world_to_grid(world_point);
		return isFree(grid_point);
	}

	//A point outside the margins of the grid will be considered as occupied
	bool isOccupied(WorldPoint world_point)
	{
		return !isFree(world_point);
	}
};

int Costmap::thresh = 20;	//Default value, updated from the parameter server

class GlobalCostmap:public Costmap
{
public:
	GlobalCostmap()
	{
		costmap_topic = "/global_costmap/costmap/costmap";
		//get_map();
	}
};

class LocalCostmap:public Costmap
{
public:
	LocalCostmap()
	{
		costmap_topic = "/local_costmap/costmap/costmap";
		subscribe();
	}
};
#endif