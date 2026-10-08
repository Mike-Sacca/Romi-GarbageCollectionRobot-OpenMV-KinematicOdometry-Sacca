#pragma once

#include <Arduino.h>
#include <LineSensor.h>

#include "chassis-params.h"

#include <utils.h>

class Chassis
{
protected:
    /**
     * You can change the control loop period, but you should use multiples of 4 ms to 
     * avoid rounding errors.
     */
    const uint16_t CONTROL_LOOP_PERIOD_MS = 20;

    /**
     * loopFlag is used to tell the program when to update. It is set when Timer4
     * overflows (see InitializeMotorControlTimer). Some of the calculations are too
     * lengthy for an ISR, so we set a flag and use that to key the calculations.
     * 
     * Note that we use in integer so we can see if we've missed a loop. If loopFlag is
     * more than 1, then we missed a cycle.
     */
    static volatile uint8_t loopFlag;

public:
    Chassis(void) {}
    void InititalizeChassis(void)
    {
        InitializeMotorControlTimer();
        InitializeMotors();
    }

    /**
     * Checks to see if we're ready do do a motor control calculation
     */
    bool CheckLoopFlag(void) { return loopFlag > 0; }

    // For setting motor control gains; Kd is unneeded
    void SetMotorKp(float kp);
    void SetMotorKi(float ki);
    void SetMotorKd(float kd);

    /* Where the bulk of the work for the motors gets done. */
    void SpinOnce(void);

    /* Needed for managing motors. */
    static void Timer4OverflowISRHandler(void);

public:
    /* Converts u,omega to wheel speeds */
    void SetTwist(float fwdSpeedCMsec, float angVelRPM);

    /**
     * A utility function for converting robot speed to wheel speed. Left 
     * public so that you can test more easily.
     */
    void SetWheelSpeeds(float, float);

    Twist CalcMotionInLocalFrame(void);

public:
    void Stop(void) {SetMotorEfforts(0, 0);}

protected:
    /**
     * Initialization and Setup routines
     */
    void InitializeMotorControlTimer(void);
    void InitializeMotors(void);
    
    void UpdateMotors(void);
    void SetMotorEfforts(int16_t, int16_t);
};
