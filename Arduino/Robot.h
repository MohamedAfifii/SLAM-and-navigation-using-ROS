#ifndef __ROBOT__
#define __ROBOT__

#include "Motor.h"

#define L 27    

struct Robot
{
    double dl, dr;
    Motor right = Motor(8, 7, 2, 3);    
    Motor left = Motor(4, 5, 18, 19);   

    //vc: Target linear velocity of the robot (cm/sec)
    //wc: Target angular velocity of the robot (rad/sec)
    //The function uses inverse kinematics to set the desired angular velocities of the two motors
    void setTarget(double vc, double wc)
    {
        right.setTargetSpeed((2*vc+wc*L) / (2.0*R));
        left.setTargetSpeed((2*vc-wc*L) / (2.0*R));
    }

    //Modifies the PWM of the left and right motors to track the desired linear and angular velocities.
    //Stores the distance covered by the left and right wheels to be published later to ROS.
    void update()
    {
        dl = left.update();
        dr = right.update(); 
    }
};

#endif
