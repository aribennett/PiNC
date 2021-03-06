#include <Motor.h>
#include <SerialClient.h>
#include <IntervalTimer.h>

MotorList motorList;
IntervalTimer motorTimer;

void Motor::setJerk(float jerk)
{
    _jerk = jerk;
}

void Motor::setAlpha(float alpha)
{
    _alpha = alpha;
    _jerk = 0;
}

void Motor::setOmega(float omega)
{
    _omega = omega;
    _alpha = 0;
    _jerk = 0;
}

void Motor::setTheta(float theta)
{
    _theta = theta;
    _omega = 0;
    _alpha = 0;
    _jerk = 0;
}

MotorStatePacket Motor::getMotorState()
{
    MotorStatePacket toSend;
    // since these values cannot grow monotonically, downsample them
    // to fit in the message bounds.
    toSend.theta = _theta;
    toSend.omega = (int16_t)(_omega*100);
    toSend.alpha = (int16_t)(_alpha*100);
    return(toSend);
}

void MotorList::addMotor(Motor* motor)
{
    _motorList[_motorCount] = motor;
    motor->setId(_motorCount);
    ++_motorCount;
}

void runMotorsAlias()
{
    motorList.runMotors();
}

void startMotorTimer()
{
    motorTimer.begin(runMotorsAlias, COMMUTATION_INTERVAL);
}

void MotorList::runMotors()
{
    for(uint8_t i = 0; i < _motorCount; ++i)
    {
        _motorList[i]->run();
    }
}