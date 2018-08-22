
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher *pubPtr;

void odom_cb(const geometry_msgs::PoseWithCovarianceStamped& msgIn) 
{
	nav_msgs::Odometry msgOut;

	msgOut.header = msgIn.header;
	msgOut.child_frame_id = "base_footprint";
	msgOut.pose = msgIn.pose;

	pubPtr->publish (msgOut);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_to_odom");

  ros::NodeHandle nh;

  ros::Subscriber odom_sub = nh.subscribe("/robot_pose_ekf/odom_combined", 100 , odom_cb);
  pubPtr = new ros::Publisher (nh.advertise<nav_msgs::Odometry>("/filtered_odom" , 100));

  ros::spin();

}
