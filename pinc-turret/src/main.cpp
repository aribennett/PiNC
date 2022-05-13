#include <StepperHFC.h>
#include <TMCConfig.h>
#include <PWMMotor.h>
#include <RCServo.h>
#include <Motor.h>
#include <NintendoExtensionCtrl.h>

#define R_SENSE 0.11f // SilentStepStick series use 0.11

TMC5160Stepper x_spi(34, R_SENSE);
StepperHFC x_driver(2, 3, 10);
TMC5160Stepper y_spi(35, R_SENSE);
StepperHFC y_driver(7, 8, 6);

PWMMotor flywheel(23);
RCServo feeder(22);
ClassicController classic;

void setup()
{
    SPI.begin();
    classic.begin();
	while (!classic.connect()) {
		Serial.println("Classic Controller not detected!");
		delay(1000);
	}
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
    static int prev_trigger = 0;
    boolean success = classic.update();
	if (success == true)
    {
        float upDown = classic.leftJoyY()-128;
        float leftRight = classic.leftJoyX()-128;
        bool rev = classic.buttonL();
        bool fire = classic.buttonR();
        if(rev && fire)
        {
            feeder.setTheta(0);
        }
        else
        {
            feeder.setTheta(90);
        }
        if (rev || fire)
        {
            flywheel.setOmega(25);
        }
        else
        {
            flywheel.setOmega(0);
        }
        x_driver.setOmega(leftRight/40);
        y_driver.setOmega(-upDown/40);
        delay(50);
	}
}
