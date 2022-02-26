#ifndef GPIO_OUTPUT
#define GPIO_OUTPUT

#include <Arduino.h>
#include <Output.h>


// Output superclass. Only to be inherited from, never to be used directly!
class GPIOOutput : public Output
{
public:
    GPIOOutput(uint16_t pin, uint16_t initialState);
    void coldStart();
    void setOutput(uint16_t output);
private:
    uint16_t _pin;
    uint16_t _state;
};

#endif