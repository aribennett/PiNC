#ifndef ADC_DATA
#define ADC_DATA

#include <Arduino.h>
#include <Data.h>


// Output superclass. Only to be inherited from, never to be used directly!
class ADCData : public Data
{
public:
    ADCData(uint16_t pin);
    void coldStart();
    void run();
private:
    uint16_t _pin;
};

#endif