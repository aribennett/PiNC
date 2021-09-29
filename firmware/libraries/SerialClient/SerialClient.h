#ifndef SERIAL_CLIENT
#define SERIAL_CLIENT
#include <arduino.h>

#define TX_TIMEOUT 1000
#define WATCHDOG_TIMEOUT 200

enum SerialCommand : uint8_t 
{
    REPORT_STATUS = 1,
    GET_STATUS = 2,
    RUN_MOTOR = 3,
    RESET = 4,
};

enum MotorCommand : uint8_t 
{
    NONE = 1,
    SET_OMEGA = 2,
    SET_ALPHA = 3,
    SET_THETA = 4,
};

struct HeaderPacket
{
    uint8_t command;
    uint8_t motorCount;
    uint8_t sensorCount;
} __attribute__ ((packed));
static_assert(sizeof(HeaderPacket) == 3, "Header packet packing issue");

struct MotorStatePacket
{
    uint8_t motorId;
    float theta;
    float omega;
} __attribute__ ((packed));
static_assert(sizeof(MotorStatePacket) == 9, "Axis packet packing issue");

struct MotorCommandPacket
{
    uint8_t motorId;
    uint8_t motorCommand;
    float control;
} __attribute__ ((packed));
static_assert(sizeof(MotorCommandPacket) == 6, "Axis packet packing issue");

struct SensorPacket
{
    uint8_t sensorID;
    int16_t sensorValue;
} __attribute__ ((packed));
static_assert(sizeof(SensorPacket) == 3, "Sensor packet packing issue");

class SerialClient
{
public:
    void coldStart();
    void run();

private:
    uint8_t _inputBuffer[64];
    uint8_t _hidMsg[64];
    uint8_t _msgLength = 0;
    HeaderPacket* _headerPointer = (HeaderPacket *)_inputBuffer; 
    uint32_t _lastRxTime = 0;
    uint32_t _lastTxTime = 0;
    void sendStatusReport();
    void checkTimeout();
    void handleInputPacket();
    void addToMessage(uint8_t* msg, uint8_t length);
};

extern SerialClient serialClient;

#endif