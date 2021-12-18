#ifndef PWM_MOTOR
#define PWM_MOTOR

#include <Arduino.h>
#include <SerialClient.h>
#include <Motor.h>

class PWMMotor: public Motor
{
public:
    PWMMotor(uint16_t en);
    void coldStart();
    void run();
    void setEnable(bool enable);

private:
    // define a dummy drive to replace later. prevents need to extend stepper library
    uint16_t _en_pin;
    uint8_t _timer_count;
};

#endif