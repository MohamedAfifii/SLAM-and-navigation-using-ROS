#ifndef __ROBOT__
#define __ROBOT__

#include "Motor.h"

#define L 27    

struct Robot
{
    double dl, dr;
    Motor left = Motor(10, 9, 3, 2);      //Hardware order: 2, 3, 9, 10
    Motor right = Motor(6, 7, 18, 19);    //Hardware order: 18, 19, 6, 7

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
