#ifndef STEPPER_HFC
#define STEPPER_HFC

#include <Arduino.h>
#include <TMCStepper.h>
#include <SerialClient.h>
#include <Motor.h>

class StepperHFC: public Motor
{
public:
    StepperHFC(const char* name, uint16_t step, uint16_t dir, uint16_t en, TMC5160Stepper *driver);
    void coldStart();
    void run();

private:
    // define a dummy drive to replace later. prevents need to extend stepper library
    TMC5160Stepper *_driver;
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