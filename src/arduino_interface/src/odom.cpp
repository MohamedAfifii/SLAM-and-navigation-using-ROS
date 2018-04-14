/*
This program computes the odometry estimation by integrating the distances moved by each wheel over time.

It subscribes to the topic "odom_arduino", from which it reads the distances covered by the left and right wheels as the x and y fields in a geometry_msgs/Vector3 message. It then updates the pose estimaition and covariance and publishes them on the "odom" topic as a nav_msgs/Odometry message.
*/

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>

#define L  0.27    //Distance between the two wheels of the robot

ros::Publisher *pubPtr;
double x = 0,y = 0,theta = 0;	//Start at the 2D pose (0,0,0).
double k = 0.05;				


void odom_arduino_cb(const geometry_msgs::Vector3& msgIn) 
{
	nav_msgs::Odometry msgOut;	
	
	double dl = msgIn.x/100, dr = msgIn.y/100;
	
	//Assume the stddev is proportional to the distance covered by the wheel.
	double sigmal = k*dl, sigmar = k*dr;	
	dl -= 3*sigmal, dr -= 3*sigmar;
	
	double dc = (dl+dr)/2;
	double newTheta = theta+(dr-dl)/L;
	newTheta = atan2(sin(newTheta), cos(newTheta));

	//You can't take the average of two angles by adding them and dividing by 2.
	//These is a work around for taking the average of 2 angles.
	double avTheta = atan2(sin(newTheta)+sin(theta), cos(newTheta)+cos(theta)); 

	x += dc*cos(avTheta);
	y += dc*sin(avTheta);
	theta = newTheta;

	//Fill the pose entries of the output message
	msgOut.pose.pose.position.x = x;
	msgOut.pose.pose.position.y = y;
	msgOut.pose.pose.position.z = 0;
    msgOut.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

	
	//Set the covariance matrix	
	/*	
	double k1 = dr*dr - dl*dl;
	double k2 = dr*dr + dl*dl;
	double k3 = dr*dl;
	
	double k4 = sigmar*sigmar + sigmal*sigmal;
	double k5 = sigmar*sigmar - sigmal*sigmal;
	
	double a = (k1+2*k3+k4)/4.0;
	double b = (k2+k5)/(2*L);
	double c = (k1-2*k3+k5)/(L*L);

	double costh = cos(avTheta);
	double sinth = sin(avTheta);
	

	msgOut.pose.covariance[0] = a*costh*costh;
	msgOut.pose.covariance[1] = a*costh*sinth;
	msgOut.pose.covariance[5] = b*costh;
	
	msgOut.pose.covariance[6] = a*costh*sinth;
	msgOut.pose.covariance[7] = a*sinth*sinth;
	msgOut.pose.covariance[11] = b*sinth;
	
	msgOut.pose.covariance[30] = b*costh;
	msgOut.pose.covariance[31] = b*sinth;
	msgOut.pose.covariance[35] = c;
	*/
	
	for(int i = 0; i < 6; i++)	msgOut.pose.covariance[7*i] = 0.001;	//Diagonal
	
	//Set the time stamp
	msgOut.header.stamp = ros::Time::now();
	
	//Publish the message
	pubPtr->publish(msgOut);
}


int main(int argc , char** argv) 
{
	ros::init (argc , argv , "odom_translator") ;
	ros::NodeHandle nh;

	pubPtr = new ros::Publisher(nh.advertise<nav_msgs::Odometry>("/odom" ,100));

	ros::Subscriber sub = nh.subscribe("/wheel_odometry" , 100 ,&odom_arduino_cb);
	ros::spin();

	delete pubPtr;
}


