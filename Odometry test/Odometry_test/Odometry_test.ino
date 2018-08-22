
#include <Encoder.h>
#include <math.h>
#include "Robot.h"

#define T 50

Robot robot;

long oldt;
double targetV = 0, targetW = 0;

void setup()
{
    Serial.begin(9600);
    oldt = millis();
}

void loop() 
{
    if(Serial.available())
    {
        String s = Serial.readStringUntil(' ');
        targetV = s.toInt();    //(cm/sec)

        s = Serial.readStringUntil('\n');
        targetW = s.toInt();    //(milli rad/sec)
        targetW /= 1000;        //(rad/sec)
    }
    
    robot.setTarget(targetV, targetW);
    
    while(millis()-oldt < T){}
    oldt = millis();
    
    robot.update();


    Serial.print(robot.x);
    Serial.print(" ");
    Serial.print(robot.y);
    Serial.print(" ");
    Serial.print(robot.theta);

    Serial.print(" ");
    Serial.print(robot.left.targetSpeed);
    Serial.print(" ");
    Serial.println(robot.right.targetSpeed);
    */

    long left = left.encoder.read();
    long right = right.encoder.read();
    Serial.print(robot.left.oldPos);
    Serial.print("   ");
    Serial.println(robot.right.oldPos);
}
