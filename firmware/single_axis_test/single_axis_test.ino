#include <StepperHFC.h>
#include "../../../interface/interface.h" // Ugly import to work around arduino issues in a shared codebase

#define R_SENSE 0.11f // SilentStepStick series use 0.11
#define PRINT_INTERVAL 20

uint32_t stall_guard = 0;

TMC5160Stepper xy1_spi(34, R_SENSE);
StepperHFC xy1_driver(2, 3, 10, &xy1_spi);
TMC5160Stepper xy2_spi(33, R_SENSE);
StepperHFC xy2_driver(4, 5, 40, &xy2_spi);

void setup()
{
  Serial.begin(115200);
  SPI.begin(); // SPI drivers
  xy1_driver.coldStart();
  xy2_driver.coldStart();
}

void loop()
{
  static uint32_t printTime = 0;
  static uint32_t commandTime = millis();
  static uint32_t startTime = millis();

  if (millis() - commandTime > 1)
  {
    xy1_driver.setOmega(cos((float)(millis() - startTime) / 1000.0)*20);
    xy2_driver.setOmega(cos((float)(millis() - startTime) / 1000.0)*20);
    commandTime = millis();
  }

  if (millis() - printTime > PRINT_INTERVAL)
  {
    ThetaOmegaAlpha xy1_status = xy1_driver.getTOA();
    Serial.print(xy1_status.theta);
    Serial.print(", ");
    Serial.print(xy1_status.omega);
    Serial.print(", ");
    Serial.println(xy1_status.alpha);
    printTime = millis();
  }
  xy1_driver.run();
  xy2_driver.run();
}
