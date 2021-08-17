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

MotorPacket Motor::getMotorState()
{
    MotorPacket toSend;
    toSend.motorCommand = STATUS;
    toSend.theta = _theta;
    toSend.omega = _omega;
    toSend.alpha = _alpha;
    toSend.motorId = _id;
    toSend.motorStatus = 1;
    strcpy(toSend.motorDescriptor, _desc);
    return(toSend);
}


void MotorList::addMotor(Motor* motor)
{
    _motorList[_motorCount] = motor;
    motor->setId(_motorCount);
    ++_motorCount;
}

void MotorList::runMotors()
{
    for(uint8_t i = 0; i < _motorCount; ++i)
    {
        _motorList[i]->run();
    }
}