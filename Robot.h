#ifndef __ROBOT__
#define __ROBOT__

#include "Motor.h"

#define L      //TODO: The distance between the two wheels in cm

struct Robot
{
    Motor left(10, 9, 3, 2);
    Motor right(6, 7, 18, 19);

    // vc: Target linear velocity of the robot (cm/sec)
    // wc: Target angular velocity of the robot (rad/sec)
    // The function uses inverse kinematics to set the desired angular velocities of the two motors
    void setTarget(double vc, double wc)
    {
        right.setTargetSpeed((2*vc+wc*L) / (2*R));
        left.setTargetSpeed((2*vc-wc*L) / (2*R));
    }

    // Modifies the PWM of the left and right motors to track the desired linear and angular velocities.
    // Returns the odometry estimated from the wheel encoder (TODO)
    void update()
    {
        double dl = left.update();
        double dr = right.update(); 
    }
};

#endif
