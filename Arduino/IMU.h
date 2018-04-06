#ifndef __IMU__
#define __IMU__

#include<Wire.h>

const int MPU_addr=0x68;
//If you need to define other constants, define them here.
const double Rad_to_Deg = 180.0 / 3.141 ;
const double Deg_to_Rad = 3.141 / 180.0 ;

struct IMU
{
    double ax, ay, az;
    double gx, gy, gz;
    double mx, my, mz;
    //If you need extra variables, define them here.
    double ax_reg, ay_reg, az_reg;
    double gx_reg, gy_reg, gz_reg;
    double mx_reg, my_reg, mz_reg;

    //When I put this function as a constructor, I get an error while trying to communicate with ROS.
    void init()
    {
        Wire.begin();
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x6B);
        Wire.write(0);
        Wire.endTransmission(true);
              
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x1B);
        Wire.write(0b00001000);
        Wire.endTransmission(true);
        
    }

    void readAcc()
    {
          Wire.beginTransmission(MPU_addr);
          Wire.write(0x3B);
          Wire.endTransmission(false);
          Wire.requestFrom(MPU_addr,6,true);
            
          ax_reg = Wire.read()<<8|Wire.read();    
          ay_reg = Wire.read()<<8|Wire.read();
          az_reg = Wire.read()<<8|Wire.read();
          
            
          ax = ( ax_reg /16384.0 - 0.02 ) * 9.8 ;
          ay = ( ay_reg /16384.0 + 0.01 ) * 9.8 ;
          az = ( az_reg /16384.0 - 0.19 ) * 9.8 ;
    }

    void readGyro()
    {
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr,6,true);
          
        gx_reg = Wire.read()<<8|Wire.read();
        gy_reg = Wire.read()<<8|Wire.read();
        gz_reg = Wire.read()<<8|Wire.read();
          
        gx = ( gx_reg / 65.5 ) * Deg_to_Rad ;
        gy = ( gy_reg / 65.5 ) * Deg_to_Rad ;
        gz = ( gz_reg / 65.5 ) * Deg_to_Rad ;         
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
