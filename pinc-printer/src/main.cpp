#include <StepperHFC.h>
#include <TMCConfig.h>
#include <SerialClient.h>
#include <Motor.h>

#define R_SENSE 0.11f // SilentStepStick series use 0.11

TMC5160Stepper x_spi(34, R_SENSE);
StepperHFC x_driver(2, 3, 10);
TMC5160Stepper y_spi(33, R_SENSE);
StepperHFC y_driver(4, 5, 40);
TMC5160Stepper z_spi(35, R_SENSE);
StepperHFC z_driver(6, 7, 39);
TMC5160Stepper a_spi(37, R_SENSE);
StepperHFC a_driver(26, 27, 41);
TMC5160Stepper b_spi(36, R_SENSE);
StepperHFC b_driver(8, 9, 38);

void setup()
{
    SPI.begin();
    serialClient.coldStart();
    coldStart5160(&a_spi, 24);
    coldStart5160(&b_spi, 24);
    coldStart2130(&z_spi, 16);
    coldStart2130(&y_spi, 16);
    coldStart2130(&x_spi, 16);
    x_driver.coldStart();
    y_driver.coldStart();
    z_driver.coldStart();
    a_driver.coldStart();
    b_driver.coldStart();
    motorList.addMotor(&x_driver);
    motorList.addMotor(&y_driver);
    motorList.addMotor(&z_driver);
    motorList.addMotor(&a_driver);
    motorList.addMotor(&b_driver);
    startMotorTimer();
}

void loop()
{
    serialClient.run();
}
