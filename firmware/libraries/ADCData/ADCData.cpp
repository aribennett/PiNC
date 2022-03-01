#include <Arduino.h>
#include <ADCData.h>

ADCData::ADCData(uint16_t pin)
{
    _pin = pin;
}

void ADCData::coldStart()
{
    pinMode(_pin, INPUT);
}

void ADCData::run()
{
    _data = analogRead(_pin);
}
