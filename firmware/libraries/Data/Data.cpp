#include <Data.h>
#include <SerialClient.h>

DataList dataList;

void DataList::addData(Data* data)
{
    _dataList[_dataCount] = data;
    data->setId(_dataCount);
    ++_dataCount;
}

void DataList::runData()
{
    for(uint8_t i = 0; i < _dataCount; ++i)
    {
        _dataList[i]->run();
    }
}

ComponentPacket Data::getData()
{
    ComponentPacket toSend;
    toSend.componentId = _id;
    toSend.value = _data;
    return(toSend);
}