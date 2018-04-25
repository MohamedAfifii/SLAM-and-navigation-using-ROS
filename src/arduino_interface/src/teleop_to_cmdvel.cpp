
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>

ros::Publisher *pubPtr;

#define abs(x) (x > 0? (x):(-x))
#define sign(x) (x > 0? 1:-1)

void cmd_cb(const geometry_msgs::Twist msgIn) 
{
	geometry_msgs::Twist msgOut;

	msgOut = msgIn;
	if(abs(msgOut.linear.x) > 1) msgOut.linear.x = 0.16*sign(msgOut.linear.x);
	msgOut.angular.z /= 3.0;

	pubPtr->publish (msgOut);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_to_cmd");

  ros::NodeHandle nh;

  ros::Subscriber cmd_sub = nh.subscribe("/teleop/cmd_vel", 100 , cmd_cb);
  pubPtr = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel" , 100));

  ros::spin();

}
