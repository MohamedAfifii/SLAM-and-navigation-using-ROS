#ifndef __MOTOR__
#define __MOTOR__

#include <Encoder.h>
#include <math.h>

#define MAXPWM 200
#define MINSPEED 2
#define TPR (128.0*75.0)  //Ticks per rev.
#define KP 5.0
#define KI 0.2
#define KD 0
#define R  3.25             


struct Motor
{
    byte pin1, pin2;    
    int pwm = 0;
    double speed, targetSpeed = 0;
    long oldt, oldPos = 0;
    double E = 0, olde = 0;

    //2, 3 are dummy values to be able to call the constructor of Encoder.
    //These values are overwritten by the values passed to the constructor of Motor.
    //You can instead modify Encoder.h and call the default constructor.
    Encoder encoder = Encoder(2,3);

    /*
     * Assign the values to pwmPin1 and pwmPin2 such that when the PWM is applied to 
     * pin1 and pin2 is set to 0 the motor moves in the forward direction.
     * 
     * Assign the values to encoderPin1 and encoderPin2 such that when the motor
     * moves forward, the encoder ticks get incremented by a positive value.
     */
    Motor(byte pwmPin1, byte pwmPin2, byte encoderPin1, byte encoderPin2)
    {
        encoder = Encoder(encoderPin1, encoderPin2);
        oldt = millis();
        pin1 = pwmPin1;
        pin2 = pwmPin2;
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
        
        digitalWrite(pin1, 0);
        digitalWrite(pin2, 0);
    }

    // Sets the desired angular velocity of the motor in rad/sec
    void setTargetSpeed(double targetSpeed)
    {
        if(abs(targetSpeed) < MINSPEED)  targetSpeed = 0;
        this->targetSpeed = targetSpeed;
    }

    // Returns the current target speed in rad/sec
    double getTargetSpeed()
    {
        return targetSpeed;  
    }

    // Returns the speed measured during the last call to the update function
    double getSpeed()
    {
        return speed;
    }


    // Computes the current speed.
    // Modifies the PWM value to track the target speed.
    // Returns the distance covered by the wheel since the last call in (cm).
    double update()
    {
        long newPos = encoder.read();
        long t = millis()-oldt;
        long ticks = newPos-oldPos;
        oldPos = newPos; 
        oldt = millis();
        
        double ticksPerSec = ticks*(1000.0/t);
        speed = (ticksPerSec/TPR)*2*M_PI;
        double e = targetSpeed - speed;
        double de = e-olde;
        olde = e;
        E += e;
        pwm += (int)(KP*e + KD*de + KI*E);

        byte pwmPin, gndPin;
        if(targetSpeed > 0)
        {
            pwm = constrain(pwm, 0, MAXPWM);
            pwmPin = pin1, gndPin = pin2;
        }
        else
        {
            pwm = constrain(pwm, -MAXPWM, 0);
            pwmPin = pin2, gndPin = pin1;
        }

        if(abs(targetSpeed) < 1) pwm = 0;
        digitalWrite(gndPin, 0);
        analogWrite(pwmPin, abs(pwm));

        return 2*M_PI*R*(ticks/TPR);
    }
};

#endif
