#include <StepperHFC.h>
#include <TMCConfig.h>
#include <PWMMotor.h>
#include <RCServo.h>
#include <SerialClient.h>
#include <Motor.h>

#define R_SENSE 0.11f // SilentStepStick series use 0.11

TMC5160Stepper x_spi(34, R_SENSE);
StepperHFC x_driver(2, 3, 10);
TMC5160Stepper y_spi(33, R_SENSE);
StepperHFC y_driver(4, 5, 40);

PWMMotor flywheel(38);
RCServo feeder(39);

void setup()
{
    SPI.begin();
    serialClient.coldStart();
    coldStart2130(&y_spi, 8);
    coldStart2130(&x_spi, 8);
    x_driver.coldStart();
    y_driver.coldStart();
    flywheel.coldStart();
    feeder.coldStart();
    motorList.addMotor(&x_driver);
    motorList.addMotor(&y_driver);
    motorList.addMotor(&flywheel);
    motorList.addMotor(&feeder);
    startMotorTimer();
}

void loop()
{
    serialClient.run();
}
