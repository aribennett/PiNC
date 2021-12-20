#include <StepperHFC.h>
#include <TMCConfig.h>
#include <SerialClient.h>
#include <Motor.h>

#define R_SENSE 0.11f // SilentStepStick series use 0.11

TMC5160Stepper x_spi(10, R_SENSE);
StepperHFC x_driver(7, 8, 9);

void setup()
{
    SPI.begin();
    serialClient.coldStart(0);
    coldStart2130(&x_spi, 8);
    x_driver.coldStart();
    motorList.addMotor(&x_driver);
    startMotorTimer();
}

void loop()
{
    serialClient.run();
}
