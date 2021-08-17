#include <SerialClient.h>
#include <Motor.h>
#include <Sensor.h>
#include <arduino.h>

SerialClient serialClient;

void SerialClient::coldStart()
{

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
    Serial.send_now();
}

void SerialClient::handleInputPacket()
{   
    // create a base pointer to the motor commands
    uint8_t motorCount = _headerPointer->motorCount;
    _lastRxTime = millis();

    MotorPacket* motorPackets = (MotorPacket*)(_inputBuffer+sizeof(HeaderPacket));
    switch (_headerPointer->command)
    {
    case REPORT_STATUS:
        // induces no action, is a client response code
        break;
    
    case GET_STATUS:
        sendStatusReport();
        break;

    case RUN_MOTOR:
        
        for(uint8_t i = 0; i < motorCount; ++i)
        {
            MotorCommand cmd = (MotorCommand)motorPackets[i].motorCommand;
            switch (cmd)
            {
            case STATUS:
                //do nothing
                break;
            
            case SET_THETA:
                //TODO implement set method
                break;
            
            case SET_OMEGA:
                motorList.getMotor(motorPackets[i].motorId)->setOmega(motorPackets[i].omega);
                break;
            
            case SET_ALPHA:
                motorList.getMotor(motorPackets[i].motorId)->setAlpha(motorPackets[i].alpha);
                break;
            
            default:
                break;
            }
        }
        sendStatusReport();
        break;
    
    case RESET:
        // TODO implement
        break;
    
    default:
        break;
    }
}

void SerialClient::checkTimeout()
{
    if(_state != IDLE && micros() - _rxStartTime > SERIAL_TIMEOUT)
    {
        _state = IDLE;
    }

    if(millis()-_lastRxTime > WATCHDOG_TIMEOUT)
    {
        // If we have no serial messages for a second, kill the system
        for(uint8_t i = 0; i < motorList.getMotorCount(); ++i)
        {
            motorList.getMotor(i)->setAlpha(0);
            motorList.getMotor(i)->setOmega(0);
        }
        _lastRxTime = millis();
    }
}

void SerialClient::run()
{
    if (Serial.available() > 0) 
    {
        if(_state == IDLE)
        {
            _rxStartTime = micros();
            _state = WAITING_FOR_HEADER;
            _messageLength = 0;
            _bufferIndex = 0;
        }

        // Add the character to the buffer
        _inputBuffer[_bufferIndex] = Serial.read();
        ++_bufferIndex;

        // If we have a header, set up the rest of the transfer
        if(_bufferIndex == sizeof(HeaderPacket))
        {
            _rxStartTime = micros();
            _state = WAITING_FOR_BODY;
            _messageLength = _headerPointer->length;
        }

        // If we have the complet message, respond
        if (_state == WAITING_FOR_BODY && _bufferIndex == _messageLength)
        {
            handleInputPacket();
            _state = IDLE;
        }
    }
    checkTimeout();
}
