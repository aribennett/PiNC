#include "RCServo.h"
#include <arduino.h>
#include <Motor.h>
#include <Servo.h>

RCServo::RCServo(uint16_t en)
{
    _en_pin = en;
}

void RCServo::coldStart()
{
    _theta = 90;
    _servo.attach(_en_pin);
}

void RCServo::run()
{
    _servo.write(_theta);
}
