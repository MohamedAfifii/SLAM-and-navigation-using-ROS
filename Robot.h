#ifndef __ROBOT__
#define __ROBOT__

#include "Motor.h"

#define L  27    

struct Robot
{
    double x = 0, y = 0, theta = 0;
    Motor left = Motor(10, 9, 3, 2);
    Motor right = Motor(6, 7, 18, 19);

    // vc: Target linear velocity of the robot (cm/sec)
    // wc: Target angular velocity of the robot (rad/sec)
    // The function uses inverse kinematics to set the desired angular velocities of the two motors
    void setTarget(double vc, double wc)
    {
        right.setTargetSpeed((2*vc+wc*L) / (2*R));
        left.setTargetSpeed((2*vc-wc*L) / (2*R));
    }

    // Modifies the PWM of the left and right motors to track the desired linear and angular velocities.
    // Publishes the odometry estimated from the wheel encoder (TODO)
    void update()
    {
        state sl = left.update();
        state sr = right.update(); 
        double dl = sl.d, dr = sr.d;
        double vl = sl.v, vr = sr.v;
        
        double dc = (dl+dr)/2;
        double vc = (vl+vr)/2;
        double wc = (vr-vl)/L;

        double newTheta = theta+(dr-dl)/L;
        newTheta = atan2(sin(newTheta), cos(newTheta));

        // You can't take the average of two angles by adding them and dividing by 2.
        // These is a work around for taking the average of 2 angles.
        double avTheta = atan2(sin(newTheta)+sin(theta), cos(newTheta)+cos(theta)); 

        x += dc*cos(avTheta);
        y += dc*sin(avTheta);
        theta = newTheta;
    }
};

#endif
