#ifndef STEPPER_HFC
#define STEPPER_HFC

#include <Arduino.h>
#include <TMCStepper.h>
#include <SerialClient.h>
#include <Motor.h>
#include <Encoder.h>

class StepperHFC: public Motor
{
public:
    StepperHFC(uint16_t step, uint16_t dir, uint16_t en);
    StepperHFC(uint16_t step, uint16_t dir, uint16_t en, Encoder* commEncoder, uint32_t ppr);
    void coldStart();
    void run();
    void setEnable(bool enable);

private:
    int32_t _step = 0;
    int32_t _encOffset = 0;
    bool _dir = false;
    uint16_t _en_pin;
    uint16_t _stepPin;
    bool _edgeState = false;
    uint16_t _dirPin;
    uint32_t _controlAccumulator = 0;
    uint32_t _stepAccumulator = 0;
    int32_t _stepCount = 0;
    bool _motorStopped = true;
    int32_t _timePeriod = 0;
    Encoder*  _commEncoder;
    int32_t _ppr = -1;
    int32_t getPhaseOffset();
    int32_t getEncoderStep();
    int32_t getStep();
};

#endif