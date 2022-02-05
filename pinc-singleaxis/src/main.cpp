#include <StepperHFC.h>
#include <TMCConfig.h>
#include <SerialClient.h>
#include <Motor.h>
#include <Encoder.h>

#define R_SENSE 0.11f // SilentStepStick series use 0.11
#define A_ENCA 23
#define A_ENCB 19

Encoder a_enc(A_ENCB, A_ENCA);
TMC5160Stepper a_spi(37, R_SENSE);
StepperHFC a_driver(26, 27, 41, &a_enc, 8192);


void setup()
{
    SPI.begin();
    coldStart5160(&a_spi, 4);
    a_driver.coldStart();
    motorList.addMotor(&a_driver);
    a_driver.setEnable(true);
    serialClient.coldStart(0);
    startMotorTimer();
}

void loop()
{
    static long ts = millis();
    serialClient.run();
    if(millis() - ts > 100)
    {
        float time =  (float)millis()/1000.0;
        a_driver.setOmega(3*sin(time));
        ts = millis();
        Serial.print(a_driver.getEncoderStep());
        Serial.print("   ");
        Serial.print(a_driver.getStep());
        Serial.print("   ");
        Serial.println(a_driver.getPhaseOffset());
    }
}
