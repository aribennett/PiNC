#include <StepperHFC.h>
#include <SerialClient.h>
#include <Motor.h>

#define R_SENSE 0.11f // SilentStepStick series use 0.11
#define PRINT_INTERVAL 200

uint32_t stall_guard = 0;

TMC5160Stepper z1_spi(34, R_SENSE);
StepperHFC z1_driver(2, 3, 10, &z1_spi, 8);
// TMC5160Stepper xy2_spi(33, R_SENSE);
// StepperHFC xy2_driver(4, 5, 40, &xy2_spi, 8);
TMC5160Stepper xy2_spi(36, R_SENSE);
StepperHFC xy2_driver(8, 9, 38, &xy2_spi, 24);
TMC5160Stepper xy1_spi(37, R_SENSE);
StepperHFC xy1_driver(26, 27, 41, &xy1_spi, 24);

void setup()
{
  SPI.begin();
  serialClient.coldStart();
  xy1_driver.coldStart();
  xy2_driver.coldStart();
  z1_driver.coldStart();
  motorList.addMotor(&xy1_driver);
  motorList.addMotor(&xy2_driver);
  motorList.addMotor(&z1_driver);
  startMotorTimer();
}


void loop()
{
  serialClient.run();
}
