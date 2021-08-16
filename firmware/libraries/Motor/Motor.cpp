#include <Motor.h>
#include <SerialClient.h>

MotorList motorList;

void Motor::setAlpha(float alpha)
{
    _alpha = alpha;
}

void Motor::setOmega(float omega)
{
    _omega = omega;
}

ThetaOmegaAlpha Motor::getTOA()
{
    ThetaOmegaAlpha status;
    status.alpha = _alpha;
    status.theta = _theta;
    status.omega = _omega;
    return (status);
}

MotorPacket Motor::getMotorState()
{
    MotorPacket toSend;
    toSend.toa = getTOA();
    toSend.motorId = _id;
    toSend.motorStatus = 1;
    strcpy(toSend.motorDescriptor, _desc);
    return(toSend);
}


void MotorList::addMotor(Motor* motor)
{
    _motorList[_motorCount] = motor;
    ++_motorCount;
}

void MotorList::runMotors()
{
    for(uint8_t i = 0; i < _motorCount; ++i)
    {
        _motorList[i]->run();
    }
}