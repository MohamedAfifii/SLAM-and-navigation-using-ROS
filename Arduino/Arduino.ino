
#include "Robot.h"
#include "IMU.h"
#include "ros_wrapper.h"

Robot robot;
IMU imu;

#define T 20
long oldt;

void setup() 
{
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(odom_pub);
    nh.advertise(imu_pub);  

    imu.init();
    
    oldt = millis();
}


void loop() 
{   
    nh.spinOnce();             
    robot.setTarget(targetV, targetW);
    
    while(millis()-oldt < T){}
    oldt = millis();

    robot.update();
    imu.update();
    
    publishState(robot, imu);
}
