/*
This program computes the odometry estimation by integrating the distances covered by each wheel over time.

It subscribes to the topic "odom_arduino", from which it reads the distances covered by the left and right wheels as the x and y fields in a geometry_msgs/Vector3 message. 

It then updates the pose estimaition and publishes it on the "/wheelodom_pose" topic as a geometry_msgs/Pose2D message (this pose estimation is only for debugging purposes).

It also computes the instantaneous linear and angular velocities as well as their covariances relative to the robot frame and publishes this information on "/wheelodom_twist" as a nav_msgs/Odometery message to be used by the robot_localizaiton package. The next step would be to send this information in a geometry_msgs/Twist message.
*/

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#define L  0.27    //Distance between the two wheels of the robot in meters.
#define abs(x)	(((x)>0)? (x):-(x))

ros::Publisher *posePubPtr, *twistPubPtr;
double x = 0,y = 0,theta = 0;	//Start at the 2D pose (0,0,0).
double k = 0.05;				


ros::Time last_time = ros::Time(0);	//ros::Time(0) is a special time value.



void odom_arduino_cb(const geometry_msgs::Vector3& msgIn) 
{
	geometry_msgs::Pose2D poseMsg;
	nav_msgs::Odometry twistMsg;
	
	ros::Time current_time = ros::Time::now();
	
	//If this is the very first measurement, skip it.
	//This is done to avoid using a bad 'dt' value.
	if(last_time == ros::Time(0))
	{
		last_time = current_time;
		return;
	}	
	double dt = (current_time - last_time).toSec();
	double dt2 = dt*dt;
	last_time = current_time;
	
	//Get the distances covered by each wheel in meters.
	double dl = msgIn.x/100, dr = msgIn.y/100;
	
	//Assume the stddev is proportional to the distance covered by the wheel.
	//You should take the absolute of the stddevs if you are going to use them for uncertainty estimation.
	double sigmal = k*dl, sigmar = k*dr;			
	dl -= 3*sigmal, dr -= 3*sigmar;					//Calibration
	//double sigmal2 = sigmal*sigmal, sigmar2 = sigmar*sigmar;
	double sigmal2 = abs(sigmal), sigmar2 = abs(sigmar);
	
	//Update pose
	double dc = (dl+dr)/2;
	double newTheta = theta+(dr-dl)/L;
	newTheta = atan2(sin(newTheta), cos(newTheta));
	x += dc*cos(theta);
	y += dc*sin(theta);
	theta = newTheta;


	//Compute linear and angular velocities
	double vc = (dr+dl)/(2*dt);
	double wc = (dr-dl)/(L*dt);
	
	
	//Fill in the pose entries
	poseMsg.x = x;
	poseMsg.y = y;
    poseMsg.theta = theta;

	
	//Fill in the twist entries
	twistMsg.twist.twist.linear.x = vc;
	twistMsg.twist.twist.angular.z = wc;
	
	//Calculate twist covariance
	//double cv2 = (sigmar2+sigmal2)/(4*dt2);
	//double cw2 = (sigmar2+sigmal2)/(L*L*dt2);
	//double cvcw = (sigmar2-sigmal2)/(2*L*dt2);
	double cv2 = (sigmar2+sigmal2)/(2*dt);
	double cw2 = (sigmar2+sigmal2)/(L*dt);
	double cvcw = (sigmar2-sigmal2)/(2*L*dt);
	
	//Set the covariance
	twistMsg.twist.covariance[0] = cv2;
	twistMsg.twist.covariance[5] = cvcw;
	twistMsg.twist.covariance[30] = cvcw;
	twistMsg.twist.covariance[35] = cw2;
	//twistMsg.twist.covariance[0] = 0.1;
	//twistMsg.twist.covariance[35] = 0.1;
	
	//Set the message header
	twistMsg.header.stamp = current_time;
	twistMsg.header.frame_id = "odom";
	twistMsg.child_frame_id = "base_link";
	
	//Publish the messages
	twistPubPtr->publish(twistMsg);
	posePubPtr->publish(poseMsg);
}


int main(int argc , char** argv) 
{
	ros::init (argc , argv , "odom_translator") ;
	ros::NodeHandle nh;

	posePubPtr = new ros::Publisher(nh.advertise<geometry_msgs::Pose2D>("/wheelodom_pose" ,100));
	twistPubPtr = new ros::Publisher(nh.advertise<nav_msgs::Odometry>("/wheelodom_twist" ,100));

	ros::Subscriber sub = nh.subscribe("/wheel_odometry" , 100 ,&odom_arduino_cb);
	ros::spin();

	delete posePubPtr;
}


