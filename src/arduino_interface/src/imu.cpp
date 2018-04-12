
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Accel.h>
#include <sensor_msgs/Imu.h>

ros::Publisher *pubPtr;

void imu_cb(const geometry_msgs::Accel& msgIn) 
{
	sensor_msgs::Imu msgOut;

	//Assuming that the imu axes are aligned with their corresponding robot axes.
	msgOut.header.frame_id = "base_link";		
	
	msgOut.linear_acceleration.x = msgIn.linear.x;
    msgOut.linear_acceleration.y = msgIn.linear.y;
	msgOut.linear_acceleration.z = msgIn.linear.z;

	msgOut.angular_velocity.x = msgIn.angular.x;
    msgOut.angular_velocity.y = msgIn.angular.y;
	msgOut.angular_velocity.z = msgIn.angular.z;
	
	msgOut.header.stamp = ros::Time::now();

	pubPtr->publish (msgOut);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_msg_sender");

  ros::NodeHandle nh;

  ros::Subscriber acc_sub = nh.subscribe("/acc_topic", 100 , imu_cb);

  pubPtr = new ros::Publisher (nh.advertise<sensor_msgs::Imu>("/imu/data_raw" , 100));

  ros::spin();

}
