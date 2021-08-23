#include <Motor.h>
#include <SerialClient.h>
#include <IntervalTimer.h>

MotorList motorList;
IntervalTimer motorTimer;

void Motor::setAlpha(float alpha)
{
    _alpha = alpha;
}

void Motor::setOmega(float omega)
{
    _omega = omega;
}

MotorPacket Motor::getMotorState()
{
    MotorPacket toSend;
    toSend.motorCommand = STATUS;
    toSend.theta = _theta;
    toSend.omega = _omega;
    toSend.alpha = _alpha;
    toSend.motorId = _id;
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