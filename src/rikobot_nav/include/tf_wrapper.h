#ifndef __TF_WRAPPER__
#define __TF_WRAPPER__

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>

#include <my_types.h>

class TFWrapper
{
	tf::TransforListener listener;
	tf::StamperTransfor transform;

	//Listens to the /tf topic and waits for a transformation between the /map and /base_footprint
	//frames for atmost 3 seconds. If no transformation is available, an exception will be raised.	
	Pose getCurrentPose()
	{
		ros::Time now = ros::Time::now();
		listener.waitForTransform("/map", "/base_footprint", now, ros::Duration(3));
		listener.lookupTransform("/map", "base_footprint", now, transform);

		Pose p;
		p.position = transform.getOrigin();
		p.orientation = transform.getRotation();

		return p;
	}
};

#endif