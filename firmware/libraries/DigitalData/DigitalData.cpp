#include <Arduino.h>
#include <DigitalData.h>

DigitalData::DigitalData(uint16_t pin)
{
    _pin = pin;
}

void DigitalData::coldStart()
{
    pinMode(_pin, INPUT_PULLUP);
}

void DigitalData::run()
{
    _data = digitalRead(_pin);
}
