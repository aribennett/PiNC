#include <Arduino.h>
#include <PWMOutput.h>

PWMOutput::PWMOutput(uint16_t pin, uint16_t initialState)
{
    analogWriteResolution(12);
    _pin = pin;
    _state = initialState;
}

void PWMOutput::coldStart()
{
    pinMode(_pin, OUTPUT);
    analogWrite(_pin, _state);
}

void PWMOutput::setOutput(uint16_t output)
{
    _state = output;
    analogWrite(_pin, _state);
}