#ifndef __TF_WRAPPER__
#define __TF_WRAPPER__

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>

#include <my_types.h>

class TFWrapper
{
public:
	tf::TransformListener listener;
	tf::StampedTransform transform;

	//Listens to the /tf topic and waits for a transformation between the /map and /base_footprint
	//frames for atmost 3 seconds. If no transformation is available, an exception will be raised.	
	Pose getCurrentPose()
	{
		listener.waitForTransform("map", "base_footprint", ros::Time::now(), ros::Duration(1));
		listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);

		geometry_msgs::TransformStamped msg;
		tf::transformStampedTFToMsg(transform, msg);

		Pose p;		
		p.position.x = msg.transform.translation.x;
		p.position.y = msg.transform.translation.y;
		p.orientation = msg.transform.rotation;

		return p;
	}
};

#endif