#pragma once

#include <event_timer.h>

#include "chassis.h"
#include "utils.h"
#include "openmv.h"

//We've commented out the IMU; put it back in, if you're using it
//#include "LSM6.h"

class Robot
{
protected:
    /**
     * We define some modes for you. SETUP is used for adjusting gains and so forth. Most
     * of the activities will run in AUTO. You shouldn't need to mess with these.
     */
    enum ROBOT_CTRL_MODE
    {
        CTRL_TELEOP,
        CTRL_AUTO,
    };
    ROBOT_CTRL_MODE robotCtrlMode = CTRL_AUTO;

    /**
     * robotState is used to track the current (low-level) state of the robot. 
     * You will add new states as the term progresses.
     */
    enum ROBOT_STATE 
    {
        ROBOT_IDLE,
        ROBOT_DRIVE_TO_POINT,
        ROBOT_SEARCHING,
        ROBOT_APPROACHING,
        ROBOT_LIFTING,
        ROBOT_WEIGHING,
        ROBOT_DEPOSITING, 
        ROBOT_ARM_RESET, 
        ROBOT_TURN_TO_FACE, 
        ROBOT_LOST_TAG, 
        ROBOT_TEST, 
    };
    ROBOT_STATE robotState = ROBOT_IDLE;

    /**
     * robotNav defines the high-level destination. It holds the current goal of the robot.
     */
    enum ROBOT_NAV
    {
        NAV_NONE,   // nothing
        NAV_PICKUP, // heading out to pick up some trash
        NAV_DUMP,   // heading to the dump
        NAV_HOME,   // heading back to start/home
    };
    ROBOT_NAV navGoal = NAV_NONE;

    /* Declare the chassis*/
    Chassis chassis;

    /* To add later: rangefinder, camera, etc.*/

    /**
     * For tracking current pose and the destination.
     */
    Pose currPose;
    Pose destPose;

    float totalTurn; 

    float tag_x_position; 
    float tag_y_position;
    float tag_width;
    float tag_height;
    float tag_id; 
    float tag_yaw;

    unsigned int TARGET_APRILTAG_ID = 15; 

    const uint8_t LED_PIN = 13;

    // For managing key presses
    String keyString;

    /**
     * Uncomment LineSensor if you want to use it as a cliff sensor
     */ 
    // LineSensor lineSensor;

    /* Timer for dead reckoning. Use sparingly. */
    EventTimer navTimer;

    // /**
    //  * Uncomment if you need to use the IMU
    //  */
    // LSM6 imu;
    // LSM6::vector<float> eulerAngles;

    /**
     * Declare a camera object
     */
    OpenMV camera;
    void HandleAprilTag(const AprilTagDatum& tag, bool gotTag);
    bool CheckApproachComplete(void);

public:
    Robot(void) {keyString.reserve(10);}
    void InitializeRobot(void);
    void RobotLoop(void);

protected:
    /* For managing IR remote key presses*/
    void HandleKeyCode(int16_t keyCode);

    /* For managing serial input */
    bool CheckSerialInput(void);
    void ParseSerialInput(void);

    /* State changes */    
    void EnterIdleState(void);

    /* Mode changes */
    void EnterTeleopMode(void);
    void EnterAutoMode(void);

    /* Distance sensing */
    void HandleDistanceReading(float);

    // /* Navigation methods.*/
    void UpdatePose(const Twist&);
    void SetDestination(const Pose& destination);

    void DriveToPoint(void);
    bool CheckReachedDestination(void);
    void HandleDestination(void);

    bool CheckAngleReached(void);
    void TurnToAngle(void); 
    void HandleAngleReached(void);

    void ApproachAprilTag(void); 
    void HandleApproachComplete(void);
    void ScanForAprilTag(void);
    bool CheckScanComplete(void);
    void HandleScanComplete(void);
    void StartTagIdBlink(unsigned int tagID);
    void UpdateLED();

};
