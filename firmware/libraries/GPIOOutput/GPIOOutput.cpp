#include <Arduino.h>
#include <GPIOOutput.h>

GPIOOutput::GPIOOutput(uint16_t pin, uint16_t initialState)
{
    _pin = pin;
    _state = initialState;
}

void GPIOOutput::coldStart()
{
    pinMode(_pin, OUTPUT);
    digitalWriteFast(_pin, _state);
}

void GPIOOutput::setOutput(uint16_t output)
{
    _state = output;
    digitalWriteFast(_pin, _state);
}
