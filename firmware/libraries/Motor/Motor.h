#ifndef MOTOR
#define MOTOR

#include <Arduino.h>
#include <SerialClient.h>


// Motor superclass. Only to be inherited from, never to be used directly!
class Motor
{
public:
    virtual void coldStart(){};
    virtual void run(){};
    void setAlpha(float alpha);
    void setOmega(float omega);
    void setId(uint8_t id){ _id=id; };
    uint8_t getId(){ return(_id); };
    ThetaOmegaAlpha getTOA();
    MotorPacket getMotorState();

protected:
    uint8_t _id;
    float _theta = 0;
    float _omega = 0;
    float _alpha = 0;
    char _desc[10];
};

class MotorList
{
    public:
        void addMotor(Motor* motor);
        Motor* getMotor(uint8_t index){return(_motorList[index]); };
        uint8_t getMotorCount(){ return(_motorCount); };
        void runMotors();
    private:
        Motor* _motorList[64];
        uint8_t _motorCount;
};

extern MotorList motorList;

#endif