#include <SerialClient.h>
#include <Motor.h>
#include <Data.h>
#include <Output.h>
#include <arduino.h>

SerialClient serialClient;

void SerialClient::coldStart(uint8_t id, bool timeout)
{
    _boardid = id;
    _timeout = timeout;
}

void SerialClient::sendStatusReport()
{
    // Buffer the header into the serial bus
    HeaderPacket header;
    _msgLength = 0;
    header.boardid = _boardid;
    header.command = REPORT_STATUS;
    header.motorCount = motorList.getMotorCount();
    header.componentCount = dataList.getDataCount();
    
    addToMessage((uint8_t*) &header, sizeof(header));
    // Print out motor states in order
    for(uint8_t i = 0; i < header.motorCount; ++i)
    {
        MotorStatePacket mp = motorList.getMotor(i)->getMotorState();
        addToMessage((uint8_t*) &mp, sizeof(mp));
    }
    // Print out inputs in order
    for(uint8_t i = 0; i < header.componentCount; ++i)
    {
        ComponentPacket cp = dataList.getData(i)->getData();
        addToMessage((uint8_t*) &cp, sizeof(cp));
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
    uint8_t outputCount = _headerPointer->componentCount;
    _lastRxTime = millis();

    MotorCommandPacket* motorPackets = (MotorCommandPacket*)(_inputBuffer+sizeof(HeaderPacket));
    ComponentPacket* outputPackets = (ComponentPacket*)(_inputBuffer+sizeof(HeaderPacket)+sizeof(MotorCommandPacket)*motorCount);
    switch (_headerPointer->command)
    {
    case REPORT_STATUS:
        // induces no action, is a client response code
        break;
    
    case GET_STATUS:
        sendStatusReport();
        break;

    case RUN:
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
        for(uint8_t i = 0; i < outputCount; ++i)
        {
            outputList.getOutput(outputPackets[i].componentId)->setOutput(outputPackets[i].value);
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
    if(millis()-_lastRxTime > WATCHDOG_TIMEOUT && _timeout)
    {
        // If we have no serial messages for a second, kill the system
        for(uint8_t i = 0; i < motorList.getMotorCount(); ++i)
        {
            motorList.getMotor(i)->setAlpha(0);
            motorList.getMotor(i)->setOmega(0);
            motorList.getMotor(i)->setJerk(0);
            motorList.getMotor(i)->setEnable(false);
        }
        for(uint8_t i = 0; i < outputList.getOutputCount(); ++i)
        {
            outputList.getOutput(i)->setOutput(0);
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
