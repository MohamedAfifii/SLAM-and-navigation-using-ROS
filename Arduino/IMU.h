#ifndef __IMU__
#define __IMU__

#include<Wire.h>

const int MPU_addr=0x68;
const double Rad_to_Deg = 180.0 / 3.141 ;
const double Deg_to_Rad = 3.141 / 180.0 ;

#define N 100.0   //Number of calibration samples 
#define D 10      //Delay between calibration samples

struct IMU
{
    double ax, ay, az;
    double gx, gy, gz;
    double mx, my, mz;

    double ax_reg, ay_reg, az_reg;
    double gx_reg, gy_reg, gz_reg;
    double mx_reg, my_reg, mz_reg;

    double ax_calibration = 0, ay_calibration = 0, az_calibration = 0; 
    double gx_calibration = 0, gy_calibration = 0, gz_calibration = 0;


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


        //Calibration
        //Take N samples while the robot is at rest.
        for (int i = 0; i < N; i++)
        {
            //Accelerometer
            Wire.beginTransmission(MPU_addr);
            Wire.write(0x3B);
            Wire.endTransmission(false);
            Wire.requestFrom(MPU_addr,6,true);
      
            ax_calibration += Wire.read()<<8|Wire.read();    
            ay_calibration += Wire.read()<<8|Wire.read();
            az_calibration += Wire.read()<<8|Wire.read();
      
            //Gyroscope
            Wire.beginTransmission(MPU_addr);
            Wire.write(0x43);
            Wire.endTransmission(false);
            Wire.requestFrom(MPU_addr,6,true);
      
            gx_calibration += Wire.read()<<8|Wire.read();
            gy_calibration += Wire.read()<<8|Wire.read();
            gz_calibration += Wire.read()<<8|Wire.read();
      
            delay(D);
        }
    
        //Take the average
        ax_calibration /= N, ay_calibration /= N, az_calibration /= N;
        gx_calibration /= N, gy_calibration /= N, gz_calibration /= N;
        
        //The expected register value when the robot is at rest on a horizontal plane should be 16384(=1g)
        az_calibration -= 16384;  
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
          
          //Calibrate the register readings then convert them to (m/s^2)
          ax = ((ax_reg - ax_calibration) / 16384.0) * 9.8;
          ay = ((ay_reg - ay_calibration) / 16384.0) * 9.8;
          az = ((az_reg - az_calibration) / 16384.0) * 9.8;
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

        //Calibrate the register readings then convert them to (rad/s)
        gx = ((gx_reg - gx_calibration) / 65.5) * Deg_to_Rad ;
        gy = ((gy_reg - gy_calibration) / 65.5) * Deg_to_Rad ;
        gz = ((gz_reg - gz_calibration) / 65.5) * Deg_to_Rad ;  
    }

    void readMag()
    {
        //TODO: Read the data from the magnetometer and update mx, my, mz.  
    }
    
    void update()
    {
        readAcc();
        readGyro();
        //readMag();
    }
};

#endif
