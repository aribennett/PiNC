#include <SerialClient.h>
#include <Motor.h>
#include <Sensor.h>
#include <arduino.h>

SerialClient serialClient;

void SerialClient::coldStart(uint8_t id)
{
    _boardid = id;
}

void SerialClient::sendStatusReport()
{
    // Buffer the header into the serial bus
    HeaderPacket header;
    _msgLength = 0;
    header.boardid = _boardid;
    header.command = REPORT_STATUS;
    header.motorCount = motorList.getMotorCount();
    header.sensorCount = 0;
    
    addToMessage((uint8_t*) &header, sizeof(header));
    // Print out motor states in order
    for(uint8_t i = 0; i < header.motorCount; ++i)
    {
        MotorStatePacket mp = motorList.getMotor(i)->getMotorState();
        addToMessage((uint8_t*) &mp, sizeof(mp));
    }

    usb_rawhid_send(_hidMsg, 0);
}

void SerialClient::addToMessage(uint8_t* msg, uint8_t length)
{
    memcpy(_hidMsg + _msgLength, msg, length);
    _msgLength += length;
}

void SerialClient::handleInputPacket()
{   
    // create a base pointer to the motor commands
    uint8_t motorCount = _headerPointer->motorCount;
    _lastRxTime = millis();

    MotorCommandPacket* motorPackets = (MotorCommandPacket*)(_inputBuffer+sizeof(HeaderPacket));
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
            case NONE:
                //do nothing
                break;
            
            case SET_THETA:
                motorList.getMotor(motorPackets[i].motorId)->setTheta(motorPackets[i].control);
                break;
            
            case SET_OMEGA:
                motorList.getMotor(motorPackets[i].motorId)->setOmega(motorPackets[i].control);
                break;
            
            case SET_ALPHA:
                motorList.getMotor(motorPackets[i].motorId)->setAlpha(motorPackets[i].control);
                break;
            
            case SET_JERK:
                motorList.getMotor(motorPackets[i].motorId)->setJerk(motorPackets[i].control);
                break;
            
            case ENABLE:
                motorList.getMotor(motorPackets[i].motorId)->setEnable(true);
                break;

            case DISABLE:
                motorList.getMotor(motorPackets[i].motorId)->setEnable(false);
                break;

            default:
                break;
            }
        }
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
    if(millis()-_lastRxTime > WATCHDOG_TIMEOUT)
    {
        // If we have no serial messages for a second, kill the system
        for(uint8_t i = 0; i < motorList.getMotorCount(); ++i)
        {
            motorList.getMotor(i)->setAlpha(0);
            motorList.getMotor(i)->setOmega(0);
            motorList.getMotor(i)->setEnable(false);
        }
        _lastRxTime = millis();
    }

    if(micros()-_lastTxTime > TX_TIMEOUT)
    {
        sendStatusReport();
        _lastTxTime = micros();
    }
}

void SerialClient::run()
{   
    // HAVE TO ADJUST TEENSY SOURCE TO ACCOUNT FOR 1 MS HANG AT 0 TIMEOUT
    if(usb_rawhid_recv(_inputBuffer, 0) > 0)
    {
        handleInputPacket();
    }
    checkTimeout();
}
