#ifndef INTERFACE
#define INTERFACE
#include <arduino.h>

struct MsgHeader
{
    uint8_t command;
    uint16_t length;
    uint16_t motorCount;
    uint16_t sensorCount;
    uint16_t messageLength;
} __attribute__ ((packed));
static_assert(sizeof(MsgHeader) == 9), "Header packet packing issue");

struct ThetaOmegaAlpha
{
    float theta;
    float omega;
    float alpha;
} __attribute__ ((packed));
static_assert(sizeof(ThetaOmegaAlpha) == 12, "TOA packet packing issue");

struct MotorPacket
{
    uint8_t axisID;
    uint8_t axisType;
    uint8_t axisStatus;
    ThetaOmegaAlpha toa;
} __attribute__ ((packed));
static_assert(sizeof(MotorPacket) == (3 + sizeof(ThetaOmegaAlpha)), "Axis packet packing issue");

struct SensorPacket
{
    uint16_t sensorID;
    uint32_t sensorValue;
} __attribute__ ((packed));
static_assert(sizeof(SensorPacket) == 5), "Sensor packet packing issue");

struct MsgFooter
{
    uint16_t checksum;
} __attribute__ ((packed));
static_assert(sizeof(MsgHeader) == 2), "Footer packet packing issue");


#endif