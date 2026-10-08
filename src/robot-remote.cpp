/**
 * robot-remote.cpp implements features of class Robot that are related to processing
 * remote control commands. It also manages modes. You might want to trim away some of 
 * the code that isn't needed later in the term.
 */
#include "robot.h"

#include <ir_codes.h>
#include <IRdecoder.h>

/**
 * IRDecoder decoder is declared extern in IRdecoder.h (for ISR purposes), 
 * so we _must_ name it decoder.
 */
#define IR_PIN 17
IRDecoder decoder(IR_PIN);

void Robot::HandleKeyCode(int16_t keyCode)
{ 
    Serial.println(keyCode);

    // Regardless of current state, if ENTER is pressed, go to idle state
    if(keyCode == ENTER_SAVE) { EnterIdleState(); keyString = "";}


    // If STOP/MODE is pressed, it toggles control mode (auto <-> teleop)
    else if(keyCode == STOP_MODE) 
    {
        if(robotCtrlMode == CTRL_AUTO) {EnterTeleopMode(); }
        else if(robotCtrlMode == CTRL_TELEOP) {EnterAutoMode(); }
        EnterIdleState(); // Idle to avoid surprises
        keyString = "";
    }

    if(robotCtrlMode == CTRL_AUTO)
    {
        switch(keyCode)
        {
            case NUM_0_10:
                SetDestination(Pose(0, 84.9, 0));
                TARGET_APRILTAG_ID = 0; 
                break;
            case NUM_1:
                SetDestination(Pose(43, 125.6, 0));
                TARGET_APRILTAG_ID = 1; 
                break;
            case NUM_2:
                SetDestination(Pose(-42.2, 206.2, 0)); 
                TARGET_APRILTAG_ID = 2; 
                break;
            case NUM_3:
                SetDestination(Pose(-40.9, 42.9, 0));
                TARGET_APRILTAG_ID = 3; 
                break;
            case NUM_4:
                SetDestination(Pose(-41.1, 125.6, 0));
                TARGET_APRILTAG_ID = 4; 
                break;
            case NUM_5:
                SetDestination(Pose(43.3, 84.9, 225));
                TARGET_APRILTAG_ID = 5; 
                break;
            case NUM_6:
                SetDestination(Pose(42.6, 206.2, 0));
                TARGET_APRILTAG_ID = 12;  
                break;
            case NUM_7:
                SetDestination(Pose(30, 0, 0.785398));
                TARGET_APRILTAG_ID = 7; 
                break;
            case NUM_8:
                SetDestination(Pose(0, 30, -1.57));
                TARGET_APRILTAG_ID = 8; 
                break;
            case NUM_9:
                SetDestination(Pose(-10, -10, 1.57));
                TARGET_APRILTAG_ID = 9; 
                break;
        }
        keyString = "";
    }

    else if(robotCtrlMode == CTRL_TELEOP)
    {
        switch(keyCode)
        {
            case UP_ARROW:
                chassis.SetTwist(10, 0);        // 10 cm/sec
                break;
            case RIGHT_ARROW:
                chassis.SetTwist(0, -0.785);    // 45 deg/sec
                break;
            case DOWN_ARROW:
                chassis.SetTwist(-10, 0);       // 10 cm/sec
                break;
            case LEFT_ARROW:
                chassis.SetTwist(0, 0.785);     // 45 deg/sec
                break;
            case ENTER_SAVE:
                chassis.SetTwist(0, 0);         // full stop
                break;
        }
    }    
}

void Robot::EnterTeleopMode(void)
{
    Serial.println("-> TELEOP");
    robotCtrlMode = CTRL_TELEOP;
    navGoal = NAV_NONE;         //cancel any navigation goals
}

void Robot::EnterAutoMode(void)
{
    Serial.println("-> AUTO");
    robotCtrlMode = CTRL_AUTO;
    navGoal = NAV_NONE;         //cancel any navigation goals
}
