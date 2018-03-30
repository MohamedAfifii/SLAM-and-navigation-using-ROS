#ifndef __IMU__
#define __IMU__

#include<Wire.h>

const int MPU_addr=0x68;
//If you need to define other constants, define them here.

struct IMU
{
    double ax, ay, az;
    double gx, gy, gz;
    double mx, my, mz;
    //If you need extra variables, define them here.

    
    IMU()
    {
        //TODO: Write here what you want to happen in void setup.
    }

    void readAcc()
    {
        //TODO: Read the data from the accelerometer and update ax, ay, az.
    }

    void readGyro()
    {
        //TODO: Read the data from the gyroscope and update gx, gy, gz.  
    }

    void readMag()
    {
        //TODO: Read the data from the magnetometer and update mx, my, mz.  
    }
    
    void update()
    {
        readAcc();
        readGyro();
        readMag();
    }
};

#endif
