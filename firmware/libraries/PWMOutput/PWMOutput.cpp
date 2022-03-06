#include <Arduino.h>
#include <PWMOutput.h>

PWMOutput::PWMOutput(uint16_t pin, uint16_t initialState)
{
    _pin = pin;
    _state = initialState;
}

void PWMOutput::coldStart()
{
    pinMode(_pin, OUTPUT);
    _pinState = LOW;
    digitalWriteFast(_pin, _pinState);
}

void PWMOutput::setOutput(uint16_t output)
{
    _state = output;
}

void PWMOutput::run()
{
    ++_timerCount;
    if(_timerCount >= _state && _pinState == HIGH)
    {
        _pinState = LOW;
        digitalWriteFast(_pin, LOW);
    }
    else if(_timerCount < _state && _pinState == LOW)
    {
        _pinState = HIGH;
        digitalWriteFast(_pin, HIGH);
    }

    if(_timerCount >= 100)
    {
        _timerCount = 0;
    }
}