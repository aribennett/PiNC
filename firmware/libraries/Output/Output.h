#ifndef OUTPUT_H
#define OUTPUT_H

#include <Arduino.h>


// Output superclass. Only to be inherited from, never to be used directly!
class Output
{
public:
    virtual void coldStart(){};
    virtual void setOutput(uint16_t output);
    void setId(uint8_t id){ _id=id; };
    uint8_t getId(){ return(_id); };
private:
    uint8_t _id;
};

class OutputList
{
    public:
        void addOutput(Output* output);
        uint8_t getOutputCount(){ return(_outputCount); };
        Output* getOutput(uint8_t index){return(_outputList[index]); };
    private:
        Output* _outputList[64];
        uint8_t _outputCount;
};

extern OutputList outputList;

#endif