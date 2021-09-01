#include "StepperHFC.h"
#include <TMCStepper.h>
#include <arduino.h>
#include <SerialClient.h>
#include <Motor.h>

#define MICROSTEPS 16
#define STALL_VALUE -10 // [-64..63]
#define STEPS_PER_REV 200
#define CONTROL_INTERVAL 200 //microseconds
#define CONTROL_BASE 1000000
#define MIN_DIVISOR 2

StepperHFC::StepperHFC(uint16_t step, uint16_t dir, uint16_t en, TMC5160Stepper *driver, uint16_t irun)
{
    _stepPin = step;
    _dirPin = dir;
    _driver = driver;
    _irun = irun;
}

void StepperHFC::coldStart()
{
    pinMode(_en_pin, OUTPUT);
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_en_pin, LOW); // Enable driver in hardware
    digitalWrite(_dirPin, _dir);

    // Enable one according to your setup
    _driver->begin();  //  SPI: Init CS pins and possible SW SPI pins
    _driver->ihold(_irun/2); // Set motor RMS current
    _driver->irun(_irun); // Set motor RMS current
    _driver->microsteps(MICROSTEPS);
    _driver->dedge(true);
    _driver->intpol(true);
}

void StepperHFC::run()
{
    _controlAccumulator += COMMUTATION_INTERVAL;
    _stepAccumulator += COMMUTATION_INTERVAL;

    // Manage control loop
    if(_controlAccumulator > CONTROL_INTERVAL)
    {
        _omega += _alpha * CONTROL_INTERVAL / 1000000.0;
        _theta = (TWO_PI * (float)_stepCount) / (STEPS_PER_REV * MICROSTEPS);

        float divisor = _omega * MICROSTEPS * STEPS_PER_REV;
        if (abs(divisor) < MIN_DIVISOR)
        {
            _motorStopped = true;
        }
        else
        {
            _motorStopped = false;
            _timePeriod = TWO_PI * (float)CONTROL_BASE / divisor;
        }
        _controlAccumulator = 0;
    }

    // commutate motor
    if (_stepAccumulator > abs(_timePeriod) && !_motorStopped)
    {
        if (((_timePeriod < 0) != _dir))
        {
            _dir = !_dir;
            digitalWriteFast(_dirPin, _dir);
        }
        digitalWriteFast(_stepPin, _edgeState);
        _edgeState = !_edgeState;
        if (!_dir)
        {
            ++_stepCount;
        }
        else
        {
            --_stepCount;
        }
        _stepAccumulator = 0;
    }
}
