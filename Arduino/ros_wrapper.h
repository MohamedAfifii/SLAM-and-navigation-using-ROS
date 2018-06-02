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
#include <rikobot_filters/sensor_readings.h>

void twistCb(const geometry_msgs::Twist& twist);

geometry_msgs::Vector3 odom_msg;
geometry_msgs::Accel imu_msg;
rikobot_filters::sensor_readings reading_msg;

ros::NodeHandle nh;
double targetV, targetW;

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", twistCb);
ros::Publisher odom_pub("/wheel_odometry", &odom_msg);
ros::Publisher imu_pub("/acc_topic", &imu_msg);
ros::Publisher reading_pub("/reading_topic", &reading_msg);

long vtime;

void twistCb(const geometry_msgs::Twist& twist)
{
    targetV = twist.linear.x*100;   //cm/sec
    targetW = twist.angular.z;
    vtime = millis();
}

void publishOdometry(const Robot &robot)
{ 
    odom_msg.x = robot.dl;
    odom_msg.y = robot.dr;

    //Update the reading_msg
    reading_msg.odom_msg = odom_msg;
    
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

    //Update the reading_msg
    reading_msg.imu_msg = imu_msg;
    
    imu_pub.publish(&imu_msg);
}


void publishState(const Robot &robot, const IMU &imu)
{
    publishOdometry(robot);
    publishIMU(imu);
    reading_pub.publish(&reading_msg);
}


#endif
