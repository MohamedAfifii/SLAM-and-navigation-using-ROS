#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <ros.h>  
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>


#include <Encoder.h>
#include <math.h>
#include "Robot.h"

#define T 50
long oldt;

Robot robot;

void cmd_cb(const geometry_msgs::Twist& cmd_msg)
{
    double targetV = cmd_msg.linear.x*100;
    double targetW = cmd_msg.angular.z;

    robot.setTarget(targetV, targetW);
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", cmd_cb);

nav_msgs::Odometry odom;
ros::Publisher odom_pub("/odom", &odom);
tf::TransformBroadcaster odom_broadcaster;


static inline geometry_msgs::Quaternion createQuaternionFromYaw(double yaw)
{
  geometry_msgs::Quaternion q;
  q.x = 0;
  q.y = 0;
  q.z = sin(yaw * 0.5);
  q.w = cos(yaw * 0.5);
  return q;
}

void setup() 
{
    oldt = millis();
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(odom_pub);
}


void loop() 
{   
    nh.spinOnce();              

    while(millis()-oldt < T){}
    oldt = millis();

    robot.update();
    ros::Time current_time = nh.now();
   

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = robot.x;
    odom_trans.transform.translation.y = robot.y;
    odom_trans.transform.translation.z = 0.0;

    geometry_msgs::Quaternion odom_quat = createQuaternionFromYaw(robot.theta);
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = robot.x;
    odom.pose.pose.position.y = robot.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = robot.vx;
    odom.twist.twist.linear.y = robot.vy;
    odom.twist.twist.angular.z = robot.vth;

    //publish the message
    odom_pub.publish(&odom);
}
