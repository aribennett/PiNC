#include <StepperHFC.h>
#include <TMCConfig.h>
#include <SerialClient.h>
#include <Motor.h>
#include <Output.h>
#include <GPIOOutput.h>
#include <ADCData.h>

#define R_SENSE 0.11f // SilentStepStick series use 0.11

#define LASER_ON 18
#define HOTEND_ON 17
#define HEATBED_ON 16
#define HOTEND_THERMISTOR 15
#define HEATBED_THERMISTOR 14
#define FAN3_ON 32
#define FAN1_ON 29
#define FAN2_ON 0
#define SPINDLE_ON 1

TMC5160Stepper x_spi(34, R_SENSE);
StepperHFC x_driver(2, 3, 10);
TMC5160Stepper y_spi(33, R_SENSE);
StepperHFC y_driver(4, 5, 40);
TMC5160Stepper z_spi(35, R_SENSE);
StepperHFC z_driver(6, 7, 39);
TMC5160Stepper e_spi(31, R_SENSE);
StepperHFC e_driver(22, 21, 20);

#define A_ENCA 23
#define A_ENCB 19
Encoder a_enc(A_ENCB, A_ENCA);
TMC5160Stepper a_spi(37, R_SENSE);
StepperHFC a_driver(26, 27, 41, &a_enc, 8192);


#define B_ENCA 28
#define B_ENCB 30
Encoder b_enc(B_ENCB, B_ENCA);
TMC5160Stepper b_spi(36, R_SENSE);
StepperHFC b_driver(8, 9, 38, &b_enc, 8192);

GPIOOutput laser(LASER_ON, HIGH);
ADCData hotendThermistor(HOTEND_THERMISTOR);

void setup()
{
    SPI.begin();
    coldStart5160(&a_spi, 12);
    coldStart5160(&b_spi, 12);
    coldStart2130(&z_spi, 16);
    coldStart2130(&y_spi, 16);
    coldStart2130(&x_spi, 16);
    coldStart2130(&e_spi, 16);
    x_driver.coldStart();
    y_driver.coldStart();
    z_driver.coldStart();
    a_driver.coldStart();
    b_driver.coldStart();
    e_driver.coldStart();
    motorList.addMotor(&x_driver);
    motorList.addMotor(&y_driver);
    motorList.addMotor(&z_driver);
    motorList.addMotor(&a_driver);
    motorList.addMotor(&b_driver);
    motorList.addMotor(&e_driver);
    serialClient.coldStart(0);

    // Outputs
    laser.coldStart();
    outputList.addOutput(&laser);

    // Inputs
    hotendThermistor.coldStart();
    dataList.addData(&hotendThermistor);

    startMotorTimer();
}

void loop()
{
    static long timestamp;
    serialClient.run();
    dataList.runData();
}
