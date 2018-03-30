#ifndef __ROS__
#define __ROS__

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif


#include <ros.h>  
#include <ros/time.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include "Robot.h"
#include "IMU.h"

struct Ros_wrapper
{
    ros::NodeHandle nh;
    //ros::Subscriber<geometry_msgs::Twist> sub = ros::Subscriber("/cmd_vel", cmd_cb);
    ros::Publisher odom_pub = ros::Publisher("/odom", &odom);
    ros::Publisher imu_pub = ros::Publisher("/imu", &imu_msg);
    
    nav_msgs::Odometry odom;
    sensor_msgs::Imu imu_msg;

    double targetV, targetW;  


    Ros_wrapper()
    {
        nh.initNode();
        //nh.subscribe(sub);
        nh.advertise(odom_pub);
        nh.advertise(imu_pub);  
    }

    void spinOnce()
    {
        nh.spinOnce();   
    }

    void cmd_cb(const geometry_msgs::Twist& cmd_msg)
    {
        targetV = cmd_msg.linear.x*100;
        targetW = cmd_msg.angular.z;
    }

    void publishOdometry(const Robot &robot)
    { 
        odom.header.stamp = nh.now();
        odom.header.frame_id = "odom";
    
        odom.pose.pose.position.x = robot.x;
        odom.pose.pose.position.y = robot.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionFromYaw(robot.theta);

        //TODO: Set the pose covariance
        
        odom_pub.publish(&odom);
    }


    // NOTE: The filters in imu_tools uses a parameter called "constant_dt", measured in seconds (default = 0.0).
    // If left at 0.0, it will use the message timestamps to compute dt. 
    // If set to a positive value, it will use a constant dt.
    void publishIMU(const IMU &imu)
    {
        geometry_msgs::Vector3 angular_velocity;
        geometry_msgs::Vector3 linear_acceleration;

        linear_acceleration.x = imu.ax;
        linear_acceleration.y = imu.ay;
        linear_acceleration.z = imu.az;
        
        angular_velocity.x = imu.gx;
        angular_velocity.y = imu.gy;
        angular_velocity.z = imu.gz;
        
        imu_msg.angular_velocity = angular_velocity;
        imu_msg.linear_acceleration = linear_acceleration;
    
        imu_pub.publish(&imu_msg);
    }
};

#endif
