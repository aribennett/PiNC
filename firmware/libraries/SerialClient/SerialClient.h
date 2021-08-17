#ifndef SERIAL_CLIENT
#define SERIAL_CLIENT
#include <arduino.h>

#define BAUD_RATE 4000000
#define DESCRIPTOR_LENGTH 5
#define SERIAL_TIMEOUT 1000
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
    STATUS = 1,
    SET_OMEGA = 2,
    SET_ALPHA = 3,
    SET_THETA = 4,
};

enum SerialState : uint8_t 
{
    IDLE = 0,
    WAITING_FOR_HEADER = 1,
    WAITING_FOR_BODY = 2,
};


struct HeaderPacket
{
    uint8_t command;
    uint16_t length;
    uint16_t motorCount;
    uint16_t sensorCount;
} __attribute__ ((packed));
static_assert(sizeof(HeaderPacket) == 7, "Header packet packing issue");

struct MotorPacket
{
    uint8_t motorId;
    uint8_t motorCommand;
    uint8_t motorStatus;
    char motorDescriptor[DESCRIPTOR_LENGTH];
    float theta;
    float omega;
    float alpha;
} __attribute__ ((packed));
static_assert(sizeof(MotorPacket) == DESCRIPTOR_LENGTH + 15, "Axis packet packing issue");

struct SensorPacket
{
    uint16_t sensorID;
    int32_t sensorValue;
    char sensorDescriptor[DESCRIPTOR_LENGTH];
} __attribute__ ((packed));
static_assert(sizeof(SensorPacket) == DESCRIPTOR_LENGTH + 6, "Sensor packet packing issue");

struct FooterPacket
{
    uint16_t checksum;
} __attribute__ ((packed));
static_assert(sizeof(FooterPacket) == 2, "Footer packet packing issue");


class SerialClient
{
public:
    void coldStart();
    void run();

private:
    // define a dummy drive to replace later. prevents need to extend stepper library
    uint8_t _inputBuffer[1000];
    SerialState _state = IDLE;
    HeaderPacket* _headerPointer = (HeaderPacket *)_inputBuffer; 
    uint32_t _bufferIndex = 0;
    uint32_t _messageLength = 0;
    uint32_t _rxStartTime = 0;
    uint32_t _lastRxTime = 0;
    void sendStatusReport();
    void checkTimeout();
    void handleInputPacket();
};

extern SerialClient serialClient;

#endif