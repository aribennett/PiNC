#ifndef RC_SERVO
#define RC_SERVO

#include <Arduino.h>
#include <SerialClient.h>
#include <Motor.h>
#include <Servo.h>

class RCServo: public Motor
{
public:
    RCServo(uint16_t en);
    void coldStart();
    void run();

private:
    // define a dummy drive to replace later. prevents need to extend stepper library
    uint16_t _en_pin;
    Servo _servo;
};

#endif