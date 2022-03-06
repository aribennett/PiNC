#include <Arduino.h>
#include <PWMOutput.h>

PWMOutput::PWMOutput(uint16_t pin, uint16_t initialState, uint16_t freq)
{
    _pin = pin;
    _state = initialState;
    analogWriteFrequency(_pin, freq);
}

void PWMOutput::coldStart()
{
    analogWriteResolution(12);
    pinMode(_pin, OUTPUT);
    analogWrite(_pin, _state);
}

void PWMOutput::setOutput(uint16_t output)
{
    _state = output;
    analogWrite(_pin, _state);
}
