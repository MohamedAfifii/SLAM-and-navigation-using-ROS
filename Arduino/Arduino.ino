
#include "ros_wrapper.h"
#include "Robot.h"
#include "IMU.h"


#define T 10
long oldt;

Robot robot;
IMU imu;
Ros_wrapper ROS;


void setup() 
{
    oldt = millis();
}


void loop() 
{   
    ROS.spinOnce();             
    robot.setTarget(ROS.targetV, ROS.targetW);
    
    while(millis()-oldt < T){}
    oldt = millis();

    robot.update();
    ROS.publishOdometry(robot);   

    imu.update();
    ROS.publishIMU(imu);
}
