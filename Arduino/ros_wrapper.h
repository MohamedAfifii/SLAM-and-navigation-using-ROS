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
//int oldCmdV, oldCmdW;

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", twistCb);
ros::Publisher odom_pub("/wheel_odometry", &odom_msg);
ros::Publisher imu_pub("/acc_topic", &imu_msg);

long vtime;

void twistCb(const geometry_msgs::Twist& twist)
{
    targetV = twist.linear.x*100;   //cm/sec
    targetW = twist.angular.z;
    vtime = millis();
    
    /*
    int cmdV = twist.linear.x*100;
    int cmdW = twist.angular.z*100;

    //if(cmdV > 10)  targetV = 16;
    //else          targetV = 0;

    int signV = cmdV > 0, signW = cmdW > 0;
    signV *= 2, signV -= 1;
    signW *= 2, signW -= 1;
    
    cmdV = abs(cmdV), cmdW = abs(cmdW);
    
    if(cmdV > oldCmdV)  targetV = signV*16;
    if(cmdV < oldCmdV)  targetV = 0;

    if(cmdW > oldCmdW)  targetW = signW*1;
    if(cmdW < oldCmdW)  targetW = 0;

    oldCmdV = cmdV, oldCmdW = cmdW;
    */
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
