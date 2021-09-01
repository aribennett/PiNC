#ifndef STEPPER_HFC
#define STEPPER_HFC

#include <Arduino.h>
#include <TMCStepper.h>
#include <SerialClient.h>
#include <Motor.h>

class StepperHFC: public Motor
{
public:
    StepperHFC(uint16_t step, uint16_t dir, uint16_t en, TMC5160Stepper *driver, uint16_t irun);
    void coldStart();
    void run();

private:
    // define a dummy drive to replace later. prevents need to extend stepper library
    TMC5160Stepper *_driver;
    int32_t _step = 0;
    bool _dir = false;
    uint16_t _en_pin;
    uint16_t _stepPin;
    bool _edgeState = false;
    uint16_t _dirPin;
    uint16_t _irun;
    uint32_t _controlAccumulator = 0;
    uint32_t _stepAccumulator = 0;
    int32_t _stepCount = 0;
    bool _motorStopped = true;
    int32_t _timePeriod = 0;
};

#endif