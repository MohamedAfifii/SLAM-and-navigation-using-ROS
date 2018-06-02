
/*
This node subscribes to /reading_topic, from which it receives data from both the IMU and the wheel encoders.
It first makes a complementary filter update step to incorporate the new IMU measurement and update the orientation estimate.
Then it makes a kalman filter update step to fuse the orientation and wheel odometry values to update the pose estimate.
*/

#include <ros/ros.h>
#include <ros/time.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Accel.h>
#include <rikobot_filters/sensor_readings.h>
#include <nav_msgs/Odometry.h>

#include "kalman_filter.h"
#include "complementary_filter.h"

#include <Eigen/Dense>
using namespace Eigen;

void call_back(const rikobot_filters::sensor_readings& msgIn);

ros::NodeHandle nh;
ros::Publisher pub = ros::Publisher(nh.advertise<nav_msgs::Odometry>("/our_odom" ,100));
ros::Subscriber sub = nh.subscribe("/reading_topic" , 100 ,&call_back);
tf::TransformBroadcaster odom_broadcaster;


int main(int argc, char** argv) 
{
	ros::init (argc, argv, "our_filter");
	ros::spin();
}


void call_back(const rikobot_filters::sensor_readings& msgIn) 
{
	//Complementary filter
	geometry_msgs::Accel imu_msg = msgIn.imu_msg;
	double z = complementary_filter(imu_msg);
	
	
	//Kalman filter 
	geometry_msgs::Vector3 wheels_feedback = msgIn.odom_msg;
	Vector3d mu = kalman_filter(wheels_feedback, z);
	
	
	//Extract the pose
	double x = mu(0), y = mu(1), theta = mu(2);
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);	

    
	//Publish the odom->base_link transformation
	ros::Time current_time = ros::Time::now();
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	odom_broadcaster.sendTransform(odom_trans);


	//Publish the pose
	nav_msgs::Odometry msgOut;
	msgOut.pose.pose.position.x = x;
	msgOut.pose.pose.position.y = y;
	msgOut.pose.pose.position.z = 0;
    msgOut.pose.pose.orientation = odom_quat;
	msgOut.header.stamp = current_time;
	msgOut.header.frame_id = "odom";
	pub.publish(msgOut);
}




