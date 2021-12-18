#include "PWMMotor.h"
#include <arduino.h>
#include <Motor.h>

PWMMotor::PWMMotor(uint16_t en)
{
    _en_pin = en;
}

void PWMMotor::coldStart()
{
    pinMode(_en_pin, OUTPUT);
    digitalWrite(_en_pin, LOW); // Enable driver in hardware
    _timer_count = 0;
}

void PWMMotor::setEnable(bool enable)
{

}

void PWMMotor::run()
{
    ++_timer_count;
    if(_timer_count > _omega)
    {
        digitalWriteFast(_en_pin, LOW);
    }
    else
    {
        digitalWriteFast(_en_pin, HIGH);
    }

    if(_timer_count >= 100)
    {
        _timer_count = 0;
    }
}
