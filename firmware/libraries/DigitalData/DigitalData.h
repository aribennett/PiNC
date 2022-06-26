#ifndef DIGITAL_DATA
#define DIGITAL_DATA

#include <Arduino.h>
#include <Data.h>


// Output superclass. Only to be inherited from, never to be used directly!
class DigitalData : public Data
{
public:
    DigitalData(uint16_t pin);
    void coldStart();
    void run();
private:
    uint16_t _pin;
};

#endif