#ifndef MOTOR
#define MOTOR

#include <Arduino.h>
#include <SerialClient.h>

#define COMMUTATION_INTERVAL 1

// Motor superclass. Only to be inherited from, never to be used directly!
class Motor
{
public:
    virtual void coldStart(){};
    virtual void run(){};
    void setAlpha(float alpha);
    void setOmega(float omega);
    void setTheta(float theta);
    void setId(uint8_t id){ _id=id; };
    uint8_t getId(){ return(_id); };
    MotorStatePacket getMotorState();

protected:
    uint8_t _id;
    volatile float _theta = 0;
    volatile float _omega = 0;
    volatile float _alpha = 0;
};

class MotorList
{
    public:
        void runMotors();
        void addMotor(Motor* motor);
        Motor* getMotor(uint8_t index){return(_motorList[index]); };
        uint8_t getMotorCount(){ return(_motorCount); };
    private:
        Motor* _motorList[64];
        uint8_t _motorCount;
};

void startMotorTimer();

extern MotorList motorList;

#endif