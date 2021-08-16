#include <SerialClient.h>
#include <Motor.h>
#include <Sensor.h>
#include <arduino.h>

SerialClient serialClient;

void SerialClient::coldStart()
{
    Serial.begin(BAUD_RATE);
}

void SerialClient::sendStatusReport()
{
    // Buffer the header into the serial bus
    HeaderPacket header;
    FooterPacket footer;
    header.command = REPORT_STATUS;
    header.motorCount = motorList.getMotorCount();
    header.sensorCount = 0;
    header.length = sizeof(HeaderPacket) + header.motorCount*sizeof(MotorPacket) + header.sensorCount*sizeof(SensorPacket) + sizeof(FooterPacket);
    Serial.write((uint8_t*) &header, sizeof(header));

    // Print out motor states in order
    for(uint8_t i = 0; i < header.motorCount; ++i)
    {
        MotorPacket mp = motorList.getMotor(i)->getMotorState();
        Serial.write((uint8_t*) &mp, sizeof(mp));
    }

    footer.checksum = 0xffff;
    Serial.write((uint8_t*) &footer, sizeof(footer));

    Serial.println(""); //Only here for legibility. Not needed.
    Serial.send_now();
}

void SerialClient::run()
{
    if (Serial.available() > 0) 
    {
        _inputBuffer[_bufferIndex] = Serial.read();
        ++_bufferIndex;

        if(_inputBuffer[_bufferIndex-1] == '\n')
        {
            sendStatusReport();
            _bufferIndex = 0;
        }
    }
}
