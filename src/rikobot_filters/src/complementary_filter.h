
//Perform a complementary_filter update step and return the current estimate of the orientation

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>

#include <cmath>
#include <Eigen/Dense>
using namespace Eigen;

static double theta = 0;
tf::TransformBroadcaster imu_broadcaster;

ros::Time prevTime = ros::Time::now();

double complementary_filter(const geometry_msgs::Accel &imu_msg)
{
	//Compute dt
	ros::Time currentTime = ros::Time::now();
	double dt = (currentTime - prevTime).toSec();
	prevTime = curretTime;
	
	//TODO: Update theta
	
	
	//Publish the imu->IMU rotation transformation (For debugging)
	geometry_msgs::TransformStamped imu_trans;
	imu_trans.header.stamp = ros::Time::now();
	imu_trans.header.frame_id = "odom";
	imu_trans.child_frame_id = "imu";
	imu_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);
	imu_broadcaster.sendTransform(imu_trans);
	return 0;
}

