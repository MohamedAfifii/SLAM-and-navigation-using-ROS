#ifndef __MOTOR__
#define __MOTOR__

#include <Encoder.h>
#include <math.h>

#define MAXPWM 130
#define MINSPEED 3
#define TPR (128.0*75.0)  //Ticks per rev.
#define KP = 5;
 
struct Motor
{
    byte pin1, pin2;    
    int pwm = 0;
    double speed, targetSpeed = 0;
    long oldt, oldPos = 0;

    // I had to modify Encoder.h to be able to call the default constructor.
    Encoder encoder;

    /*
     * Assign the values to pwmPin1 and pwmPin2 such that when the PWM is applied to 
     * pin1 and pin2 is set to 0 the motor moves in the forward direction.
     * 
     * Assign the values to encoderPin1 and encoderPin2 such that when the motor
     * moves forward, the encoder ticks get incremented by a positive value.
     */
    Motor(byte pwmPin1, byte pwmPin2, byte encoderPin1, byte encoderPin2)
    {
        encoder.setPins(encoderPin1, encoderPin2);
        oldt = millis();
        pin1 = pwmPin1;
        pin2 = pwmPin2;
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
        
        digitalWrite(pin1, 0);
        digitalWrite(pin2, 0);
    }

    void setSpeed(double targetSpeed)
    {
        if(abs(targetSpeed) < MINSPEED)  targetSpeed = 0;
        this->targetSpeed = targetSpeed;
    }


    // Modifies the PWM value to track the target speed.
    // Returns the number of ticks since the last call.
    int update()
    {
        long newPos = encoder.read();
        long t = millis()-oldt;
        long ticks = newPos-oldPos;
        oldPos = newPos; 
        oldt = millis();
        
        double ticksPerSec = ticks*(1000.0/t);
        speed = (ticksPerSec/TPR)*2*M_PI;
        double e = targetSpeed - speed;
        pwm += (int)(KP*e);

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

        return ticks;
    }
};

#endif
