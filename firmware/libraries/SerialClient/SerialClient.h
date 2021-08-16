#ifndef SERIAL_CLIENT
#define SERIAL_CLIENT
#include <arduino.h>

#define BAUD_RATE 4000000
#define DESCRIPTOR_LENGTH 5

enum SerialCommand : uint8_t 
{
    REPORT_STATUS = 1,
    GET_STATUS = 2,
    RUN_MOTOR = 3,
    RESET = 4,
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
    uint8_t motorStatus;
    char motorDescriptor[DESCRIPTOR_LENGTH];
    float theta;
    float omega;
    float alpha;
} __attribute__ ((packed));
static_assert(sizeof(MotorPacket) == DESCRIPTOR_LENGTH + 14, "Axis packet packing issue");

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
    uint32_t _bufferIndex = 0;
    void sendStatusReport();
};

extern SerialClient serialClient;

#endif