#ifndef __ROS___
#define __ROS___

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif


#include <ros.h>  
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Accel.h>

void twistCb(const geometry_msgs::Twist& twist);

geometry_msgs::Vector3 odom_msg;
geometry_msgs::Accel imu_msg;
ros::NodeHandle nh;
double targetV, targetW;  

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", twistCb);
ros::Publisher odom_pub("/wheel_odometry", &odom_msg);
ros::Publisher imu_pub("/acc_topic", &imu_msg);

void twistCb(const geometry_msgs::Twist& twist)
{
    targetV = twist.linear.x*100;
    targetW = twist.angular.z;    
}

void cmd_cb(const geometry_msgs::Twist& cmd_msg)
{
    targetV = cmd_msg.linear.x*100;
    targetW = cmd_msg.angular.z;
}

void publishOdometry(const Robot &robot)
{ 
    odom_msg.x = robot.dl;
    odom_msg.y = robot.dr;

    odom_pub.publish(&odom_msg);
}

void publishIMU(const IMU &imu)
{
    geometry_msgs::Vector3 angular;
    geometry_msgs::Vector3 linear;

    linear.x = imu.ax;
    linear.y = imu.ay;
    linear.z = imu.az;
    
    angular.x = imu.gx;
    angular.y = imu.gy;
    angular.z = imu.gz;
    
    imu_msg.angular = angular;
    imu_msg.linear = linear;

    imu_pub.publish(&imu_msg);
}

void publishState(const Robot &robot, const IMU &imu)
{
    publishOdometry(robot);
    publishIMU(imu);
}


#endif
