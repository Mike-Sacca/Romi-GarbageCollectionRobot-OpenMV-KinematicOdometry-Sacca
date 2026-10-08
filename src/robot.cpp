#include "robot.h"
#include <IRdecoder.h>
#include "serial_comm.h"

bool ledBlinking = false;
bool ledOn = false; // current LED on/off

unsigned long ledPrevCycle = 0; // time of last toggle
int ledCyclesLeft = 0; // number of toggles left
const unsigned long LED_TOGGLE_PERIOD_MS = 500;  // .4s full cycle

void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();

    /**
     * Initialize the IR decoder. Declared extern in IRdecoder.h; see robot-remote.cpp
     * for instantiation and setting the pin.
     */
    decoder.init();

    /**
     * Put the lineSensor back in if you need is (say, for cliff detection)
     */
    // lineSensor.Initialize();

    /**
     * If using the Serial connection to the camera, initialize Serial1.
     * 
     * (TODO: Comment this out if using I2C)
     */
    Serial1.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}

void Robot::EnterIdleState(void)
{
    chassis.Stop();

    Serial.println("-> IDLE");
    robotState = ROBOT_IDLE;
    navGoal = NAV_NONE;
}

void Robot::StartTagIdBlink(unsigned int tagId)
{
    ledBlinking= true; // the led has begun blinking
    ledOn = false; // start off
    digitalWrite(LED_PIN, ledOn); // start off
    ledPrevCycle = millis(); // reset timer
    ledCyclesLeft = tagId * 2; // one full cycle per led blink
}

void Robot::UpdateLED()
{
    if (ledBlinking) {

    unsigned long now = millis();

    if (now - ledPrevCycle >= LED_TOGGLE_PERIOD_MS) {

        ledPrevCycle = now;

        // Toggle LED on/off
        ledOn = !ledOn;
        digitalWrite(LED_PIN, ledOn);

        ledCyclesLeft -= 1; // one cycle has passed

        if (ledCyclesLeft <= 0) { // when the amount of cycles left is 0, stop

            ledBlinking = false; // led is no longer blinking
            ledOn = true;

            digitalWrite(LED_PIN, LOW);
        }
     }
    }
}

/**
 * The main loop for your robot. Process both synchronous events (motor control),
 * and asynchronous events (IR presses, distance readings, etc.).
*/
void Robot::RobotLoop(void) 
{
    /**
     * Handle any IR remote keypresses.
     */
    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) HandleKeyCode(keyCode);

    /**
     * Check if time to run the chassis loop, which handles low-level motion control.
     */
    if(chassis.CheckLoopFlag()) // 20 ms timer
    {
        /**
         * SpinOnce does a motor control update
         */
        chassis.SpinOnce();

        /**
         * You can put post motor update tasks here
         */

        // We do FK regardless of state
        Twist speedInLocalFrame = chassis.CalcMotionInLocalFrame();
        UpdatePose(speedInLocalFrame);

        /**
         * Here, we break with tradition and only call these functions if we're in the 
         * DRIVE_TO_POINT state. CheckReachedDestination() is expensive, so we don't want
         * to do all the maths when we don't need to.
         *  
         * While we're at it, we'll toss DriveToPoint() in, as well.
         */ 
        if(robotState == ROBOT_DRIVE_TO_POINT)
        {
            DriveToPoint();
            if(CheckReachedDestination()) HandleDestination();
        }

        if(robotState == ROBOT_TURN_TO_FACE)
        {
            TurnToAngle();
            if(CheckAngleReached()) HandleAngleReached();
        }

        if(robotState == ROBOT_APPROACHING)
        {
            ApproachAprilTag();
            if(CheckApproachComplete()) HandleApproachComplete();
        }

        if(robotState == ROBOT_SEARCHING)
        {
            ScanForAprilTag();
            if(CheckScanComplete()) HandleScanComplete();
        }

    }

    /**
     * Check the camera for april tags. Note that we _check_ regardless of state,
     * so that we don't end up with "stale" tag readings.
     * 
     * In the handler, we process tags, based on state
     */
    AprilTagDatum tag;
    bool gotTag = camera.checkUART(tag);
    HandleAprilTag(tag, gotTag);   // and modify the handler accordingly

    UpdateLED(); 

    /**
     * If you want to use the IMU for anything, you can do that here.
     */
    // TODO: IMU stuff, if needed

    /** 
     * The code below allows you to adjust gains using the Serial Monitor.
     * CheckSerialInput() returns true when it gets a complete string, which is
     * denoted by a newline character ('\n'). 
     * 
     * WARNING: Be sure to set your Serial Monitor to append a newline!!!!
     * 
     * When you're done tuning, you can comment out or just remove the line below 
     * to save ~1.5k of program memory.
     */

     // if(robotState == ROBOT_TEST){

        //     static const float TURN_TARGET = 2.0f * PI;  // 360 deg
        //     static const float TURN_TOL    = 0.05f;      // ~3 deg

        //     chassis.SetWheelSpeeds(9.1106, 18.2212);

        //     if (fabsf(totalTurn) >= fabsf(TURN_TARGET - TURN_TOL)) {
        //         chassis.SetWheelSpeeds(0, 0);
        //         Serial.println("Stopped after 360 degrees");
        //         robotState = ROBOT_IDLE;  
        //         totalTurn = 0.0f; 
        //         } 
        //     }
    if(CheckSerialInput()) ParseSerialInput();

    } 
