#include "StepperHFC.h"
#include <TMCStepper.h>
#include <arduino.h>
#include <SerialClient.h>
#include <Motor.h>
#include <Encoder.h>

#define MICROSTEPS 16
#define STEPS_PER_REV 200
#define CONTROL_INTERVAL 200 //microseconds
#define CONTROL_BASE 1000000
#define MIN_DIVISOR 2

StepperHFC::StepperHFC(uint16_t step, uint16_t dir, uint16_t en)
{
    _stepPin = step;
    _dirPin = dir;
    _en_pin = en;
}

StepperHFC::StepperHFC(uint16_t step, uint16_t dir, uint16_t en, Encoder* commEncoder, uint32_t ppr)
{
    _stepPin = step;
    _dirPin = dir;
    _en_pin = en;
    _commEncoder = commEncoder;
    _ppr = ppr;
}

void StepperHFC::coldStart()
{
    pinMode(_en_pin, OUTPUT);
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_dirPin, _dir);
    setEnable(true);

    // zero out servo offset. Needs a delay to settle:
    if(_ppr != -1)
    {
        delay(100);
        _commEncoder->write(0);
    }
}

void StepperHFC::setEnable(bool enable)
{
    _enable = enable;
    digitalWrite(_en_pin, !enable); // Enable driver in hardware
}

int32_t StepperHFC::getStep()
{
    return(_step);
}

int32_t StepperHFC::getEncoderStep()
{
    int32_t encPos = _commEncoder->read();
    int32_t encStep = (MICROSTEPS * STEPS_PER_REV * encPos)/_ppr;
    return(encStep);
}

int32_t StepperHFC::getPhaseOffset()
{
    int32_t targetStep = _stepCount; 
    if(_ppr != -1)
    {
        targetStep = constrain(targetStep, getEncoderStep() - MICROSTEPS, getEncoderStep() + MICROSTEPS);
    }
    return(targetStep);
} 

void StepperHFC::run()
{
    _controlAccumulator += COMMUTATION_INTERVAL;
    _stepAccumulator += COMMUTATION_INTERVAL;

    // Manage control loop
    if(_controlAccumulator > CONTROL_INTERVAL)
    {
        _alpha += _jerk * CONTROL_INTERVAL / 1000000.0;
        _omega += _alpha * CONTROL_INTERVAL / 1000000.0;
        if(_ppr == -1)
        {
            _theta = (TWO_PI * (float)_stepCount) / (STEPS_PER_REV * MICROSTEPS);
        }
        else
        {
            _theta = (TWO_PI * (float)_commEncoder->read())/(float)_ppr;
        }

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
    if (_stepAccumulator > abs(_timePeriod) && !_motorStopped && _enable)
    {
        if ((_timePeriod > 0))
        {
            ++_stepCount;
        }
        else
        {
            --_stepCount;
        }
        _stepAccumulator = 0;
    }

    // Limit the step by the encoder position in this block. Limit the nominal
    // Set targetStep to the highest possible number, that is within commutation distance of the
    // stepper motor. For 16th step, that is +- 32 steps from rotor actual
    int32_t targetStep = getPhaseOffset();
    if(targetStep != _step)
    {
        bool target_dir = false;
        if(targetStep < _step)
        {
            target_dir = true;
        }

        // can only switch 1 pin per cycle
        if(target_dir != _dir)
        {
            _dir = target_dir;
            digitalWriteFast(_dirPin, _dir);
        }
        else
        {
            if(targetStep > _step)
            {
                ++_step;
            }
            else
            {
                --_step;
            }
            digitalWriteFast(_stepPin, _edgeState);
            _edgeState = !_edgeState;
        }
    }
}
