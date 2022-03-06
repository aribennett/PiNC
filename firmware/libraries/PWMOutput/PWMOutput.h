#ifndef PWM_OUTPUT
#define PWM_OUTPUT

#include <Arduino.h>
#include <Output.h>


// Output superclass. Only to be inherited from, never to be used directly!
class PWMOutput : public Output
{
public:
    PWMOutput(uint16_t pin, uint16_t initialState);
    void run();
    void coldStart();
    void setOutput(uint16_t output);
private:
    uint16_t _pin;
    uint16_t _state;
    uint16_t _pinState;
    uint16_t _timerCount = 0;
};

#endif