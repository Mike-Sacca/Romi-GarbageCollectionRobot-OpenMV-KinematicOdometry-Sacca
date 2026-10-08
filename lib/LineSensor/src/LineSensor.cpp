#include "LineSensor.h"

/**
 * Set as a threshold for "dark" in Lab 2. 500 is probably a bad choice.
 */
#define DARK_THRESHOLD 500;

/**
 * Initalize() sets pins to INPUT
 */
void LineSensor::Initialize(void)
{
    pinMode(leftSensorPin, INPUT);
    pinMode(rightSensorPin, INPUT);
}

/**
 * Calculates the error (deviation) w.r.t line.
 */
int16_t LineSensor::CalcError(void) 
{ 
    /**
     * TODO: Implement an error calculation.
     */
    return 0; 
}

/**
 * Checks for the _arrival_ at an intersection.
 */
bool LineSensor::CheckIntersection(void)
{
    /**
     * TODO: Implement a proper intersection checker
     */
    bool retVal = false;

    return retVal;
}