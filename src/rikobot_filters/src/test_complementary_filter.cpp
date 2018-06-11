
#include <ros/ros.h>
#include <ros/time.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Accel.h>
#include <rikobot_filters/sensor_readings.h>


void call_back(const rikobot_filters::sensor_readings& msgIn);

int main(int argc, char** argv) 
{
	ros::init (argc, argv, "our_filter");
	ros::spin();
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/reading_topic" , 100 ,&call_back);
}

#include "complementary_filter.h"

void call_back(const rikobot_filters::sensor_readings& msgIn) 
{
	//Complementary filter
	geometry_msgs::Accel imu_msg = msgIn.imu_msg;
	double z = complementary_filter(imu_msg);
}




