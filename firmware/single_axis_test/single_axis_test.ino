#include <StepperHFC.h>
#include <SerialClient.h>
#include <Motor.h>

#define R_SENSE 0.11f // SilentStepStick series use 0.11
#define PRINT_INTERVAL 200

uint32_t stall_guard = 0;

TMC5160Stepper xy1_spi(34, R_SENSE);
StepperHFC xy1_driver(2, 3, 10, &xy1_spi);
TMC5160Stepper xy2_spi(33, R_SENSE);
StepperHFC xy2_driver(4, 5, 40, &xy2_spi);

void setup()
{
  SPI.begin();
  serialClient.coldStart();
  xy1_driver.coldStart();
  xy2_driver.coldStart();
  motorList.addMotor(&xy1_driver);
  motorList.addMotor(&xy2_driver);
}


void loop()
{
  serialClient.run();
  motorList.runMotors();
}
