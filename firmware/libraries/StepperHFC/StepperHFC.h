#ifndef STEPPER_HFC
#define STEPPER_HFC

#include <Arduino.h>
#include <TMCStepper.h>
#include "../../../../interface/interface.h" // Ugly import to work around arduino issues in a shared codebase

class StepperHFC
{
public:
    StepperHFC(uint16_t step, uint16_t dir, uint16_t en, TMC5160Stepper *driver);
    void coldStart();
    void run();
    void setAlpha(float alpha);
    void setOmega(float omega);
    ThetaOmegaAlpha getTOA();

private:
    // define a dummy drive to replace later. prevents need to extend stepper library
    TMC5160Stepper *_driver;
    float _theta = 0;
    float _omega = 0;
    float _alpha = 0;
    int32_t _step = 0;
    bool _dir = false;
    uint16_t _en_pin;
    uint16_t _step_pin;
    bool _edge_state = false;
    uint16_t _dir_pin;
    uint32_t _last_commutate_time = ARM_DWT_CYCCNT;
    uint32_t _last_control_time = 0;
    uint32_t _step_accumulator = 0;
    int32_t _step_count = 0;
    bool _motor_stopped = true;
    int32_t _time_period = 0;


};

#endif