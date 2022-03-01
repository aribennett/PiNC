#ifndef DATA_H
#define DATA_H

#include <Arduino.h>
#include <SerialClient.h>


// Output superclass. Only to be inherited from, never to be used directly!
class Data
{
public:
    virtual void coldStart(){};
    virtual void run();
    ComponentPacket getData();
    void setId(uint8_t id){ _id=id; };
    uint8_t getId(){ return(_id); };
protected:
    uint16_t _data;
    uint8_t _id;
};

class DataList
{
public:
    void addData(Data* data);
    uint8_t getDataCount(){ return(_dataCount); };
    Data* getData(uint8_t index){return(_dataList[index]); };
    void runData();
private:
    Data* _dataList[64];
    uint8_t _dataCount = 0;
};

extern DataList dataList;

#endif