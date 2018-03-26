#ifndef __ROBOT__
#define __ROBOT__

#include "Motor.h"

#define L
#define W
#define R

struct Robot
{
    Motor left(10, 9, 3, 2);
    Motor right(6, 7, 18, 19);

    void setTarget(double v, double w)
    {
          
    }

    // Modifies the PWM of the left and right motors to track the linear and angular velocities.
    // Returns the odometry estimated from the wheel encoder (TODO)
    void update()
    {
        int leftTicks = left.update();
        int rightTicks = right.update(); 
    }
};

#endif
