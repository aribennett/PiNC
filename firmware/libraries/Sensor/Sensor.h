#ifndef SENSOR
#define SENSOR

#include <Arduino.h>

class Sensor
{
public:
    Sensor(){};
    void coldStart();
    void run();
    int16_t getReading();
private:
};


#endif