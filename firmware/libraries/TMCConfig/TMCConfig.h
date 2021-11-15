// This is a workaround due to polymorphism issues with a library I do not want to re-implement
#ifndef TMC_CONFIG
#define TMC_CONFIG
#include <TMCStepper.h>
#define MICROSTEPS 16

void coldStart5160(TMC5160Stepper* driver, uint16_t irun)
{
    driver->begin();  //  SPI: Init CS pins and possible SW SPI pins
    driver->ihold(irun/2); // Set motor RMS current
    driver->irun(irun); // Set motor RMS current
    driver->microsteps(MICROSTEPS);
    driver->dedge(true);
    driver->intpol(true);
}

void coldStart2130(TMC5160Stepper* driver, uint16_t irun)
{
    driver->begin();  //  SPI: Init CS pins and possible SW SPI pins
    driver->ihold(irun/2); // Set motor RMS current
    driver->irun(irun); // Set motor RMS current
    driver->microsteps(MICROSTEPS);
    driver->dedge(true);
    driver->intpol(true);
}

#endif