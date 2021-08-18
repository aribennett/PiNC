#include "StepperHFC.h"
#include <TMCStepper.h>
#include <arduino.h>
#include <SerialClient.h>

#define MICROSTEPS 16
#define STALL_VALUE -10 // [-64..63]
#define STEPS_PER_REV 200
#define CONTROL_INTERVAL 500 //microseconds

StepperHFC::StepperHFC(uint16_t step, uint16_t dir, uint16_t en, TMC5160Stepper *driver)
{
    _step_pin = step;
    _dir_pin = dir;
    _driver = driver;
}

void StepperHFC::coldStart()
{
    pinMode(_en_pin, OUTPUT);
    pinMode(_step_pin, OUTPUT);
    pinMode(_dir_pin, OUTPUT);
    digitalWrite(_en_pin, LOW); // Enable driver in hardware
    digitalWrite(_dir_pin, _dir);

    // Enable one according to your setup
    _driver->begin();  //  SPI: Init CS pins and possible SW SPI pins
    _driver->toff(5);  // Enables driver in software
    _driver->irun(12); // Set motor RMS current
    _driver->microsteps(MICROSTEPS);
    _driver->dedge(true);
    _driver->intpol(true);
}

void StepperHFC::run()
{
    if (micros() - _last_control_time > CONTROL_INTERVAL)
    {
        _omega += _alpha * CONTROL_INTERVAL / 1000000.0;
        _theta = (TWO_PI * (float)_step_count) / (STEPS_PER_REV * MICROSTEPS);

        float divisor = _omega * MICROSTEPS * STEPS_PER_REV;
        if (abs(divisor) < 20)
        {
            _motor_stopped = true;
        }
        else
        {
            _motor_stopped = false;
            _time_period = TWO_PI * (float)F_CPU_ACTUAL / divisor;
        }
        _last_control_time = micros();
    }
    // commutate motor
    _step_accumulator += (ARM_DWT_CYCCNT - _last_commutate_time);
    _last_commutate_time = ARM_DWT_CYCCNT;

    if (_step_accumulator > abs(_time_period) && !_motor_stopped)
    {
        if (((_time_period < 0) != _dir))
        {
            _dir = !_dir;
            digitalWriteFast(_dir_pin, _dir);
        }
        digitalWriteFast(_step_pin, _edge_state);
        _edge_state = !_edge_state;
        _step_accumulator = 0;
        if (!_dir)
        {
            ++_step_count;
        }
        else
        {
            --_step_count;
        }
    }
}
