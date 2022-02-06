#include <StepperHFC.h>
#include <TMCConfig.h>
#include <SerialClient.h>
#include <Motor.h>
#include <Encoder.h>

#define R_SENSE 0.11f // SilentStepStick series use 0.11
#define A_ENCA 23
#define A_ENCB 19

#define B_ENCA 28
#define B_ENCB 30
Encoder a_enc(B_ENCB, B_ENCA);
TMC5160Stepper a_spi(36, R_SENSE);
StepperHFC a_driver(8, 9, 38, &a_enc, 8192);


void setup()
{
    SPI.begin();
    coldStart5160(&a_spi, 8);
    a_driver.coldStart();
    motorList.addMotor(&a_driver);
    a_driver.setEnable(true);
    serialClient.coldStart(0, false);
    startMotorTimer();
}

void loop()
{
    serialClient.run();
    float time =  (float)millis()/1000.0;
    a_driver.setOmega(3*sin(time));
}
