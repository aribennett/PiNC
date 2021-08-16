#ifndef SENSOR
#define SENSOR

#include <Arduino.h>

class Sensor
{
public:
    Sensor(){};
    void coldStart();
    void run();
    float getReading();
private:
};


#endif