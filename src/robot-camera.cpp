#include "robot.h"

// ========== Checker Tolerances ==========
const float headingTolerance = 0.0; 
const float distanceTolerance = 0.75; 

// ========== Camera Instantiations ==========
OpenMV camera; 
AprilTagDatum lastTag;
bool tagInframe = false; // true if there is a tag in frame

// ========== Lost Tag Declarations ==========
const float SEARCH_ANG_SPEED = 0.4f; // rad/s
const int searchSpinDir = +1; // default CCW
const int relocateSpinDir = -1; // set when we enter LOST_TAG
const int sweepCount = 1; // number of full sweeps to do before giving up
const float maxAngVel = 1.0f; // rad/s

bool targetTagSeen = false; // have we ever seen the target recently
unsigned long lastTargetSeenMs = 0; // last time TARGET_APRILTAG_ID was seen
const unsigned long TAG_LOST_TIMEOUT_MS = 1000; // 1 second

// ========== Alignment Constants ==========
float distanceToTagCM = 0.0f; // calculated distance to tag in cm
float targetDistance = 6.5f; // target distance to tag in cm
float distanceTol = 0.5f; // tolerance for distance to tag in cm

// ========== Proportional Camera Control Constants ==========
float sumErrorX = 0;
float lastErrorX = 0;
float errorX = 0; // image center is at 80 pixels
float ang_kP = 0.03f; // proportional gain
float ang_kI = 0.00; // integral gain
float ang_kD = 0.0005f; // derivative gain
float fwd_kP = 1.0f; // forward proportional gain
float deltaErrorX = 0;
const float maxIntegral = 200.0f;
float angVel = 0; // negative because positive angular velocity turns robot left

// ========== APPROACH LOGIC ==========

void Robot::ApproachAprilTag(void)
{
/**
* TODO: Turn on the LED when the Romi begins approaching. For extra points,
* blink out a number that corresponds to the tag ID (must be non-blocking!).
* Be sure to add code (elsewhere) to turn the LED off when the Romi is
* done aligning.
*/

if (!tagInframe) {

    chassis.SetTwist(0.0f, 0.0f);
            
    return;

}

if (tag_height > 0.0f) {
    distanceToTagCM = (5.0f * 120.0f) / tag_height; // (focal length * total pixel height) / perceived object height in pixels
}

Serial.print("Distance to Tag (cm): ");
Serial.println(distanceToTagCM);

        //Proportional control to align with tag

        errorX = -1*( 80.0f - tag_x_position);  

        if (fabsf(errorX) < 2.0f) errorX = 0.0f; // to prevent jitter, consider small errors as zero

        sumErrorX += errorX;
        deltaErrorX = errorX - lastErrorX;
        lastErrorX = errorX;
                      
        if (sumErrorX >  maxIntegral) sumErrorX =  maxIntegral; // anti-windup limit
        if (sumErrorX < -maxIntegral) sumErrorX = -maxIntegral;


        angVel = (ang_kP * errorX) + (ang_kI * sumErrorX) + (ang_kD * deltaErrorX);

        if (angVel >  maxAngVel) angVel =  maxAngVel; //limiter on wheel speed
        if (angVel < -maxAngVel) angVel = -maxAngVel; //limiter on wheel speed

        float distanceError = distanceToTagCM - targetDistance;
        float fwdSpeed = fwd_kP * distanceError; // simple proportional control

        if (fwdSpeed > 15.0f) fwdSpeed = 15.0f; // limit forward speed
        if (fwdSpeed < -15.0f) fwdSpeed = -15.0f; // limit backward speed

        chassis.SetTwist(-fwdSpeed, angVel); // move forward while adjusting angular velocity

}

// ========== SCAN LOGIC ==========

void Robot::ScanForAprilTag(void)
{

    if (robotState == ROBOT_SEARCHING){

    chassis.SetTwist(0.0f, searchSpinDir * SEARCH_ANG_SPEED);

    }
}

// ========== CHECKERS ==========

bool Robot::CheckScanComplete()
{

    if (robotState == ROBOT_SEARCHING){

    bool retVal = false; 

    static float scanHeading = currPose.theta;

    float angleSearched = currPose.theta - scanHeading;
    
    const float fullSweepAngle = sweepCount * 2 * PI; 

    if (fabsf(angleSearched) >= fullSweepAngle)
    {
        retVal = true;
    }

    if (tagInframe == true)
    {
        retVal = true;
    }

    return retVal;

    }

    return false;
}

/** Note that the tolerances are in integers, since the camera works
* in integer pixels. If you have another method for calculations,
* you may need floats.
*/
bool Robot::CheckApproachComplete(void)
{
    bool retVal = false;

    float distError = distanceToTagCM - targetDistance;

    if (fabsf(distError) <= distanceTol){

        retVal = true; 
    }

    return retVal;
}

// ========== HANDLERS ==========

void Robot::HandleAprilTag(const AprilTagDatum& tag, bool gotTag)
{
    unsigned long currentTime = millis();
    bool targetThisFrame = gotTag && (tag.id == TARGET_APRILTAG_ID);


    if (targetThisFrame) {
        // Copy the data into variables available to the robot class
        tag_x_position = tag.cx; 
        tag_y_position = tag.cy;
        tag_width = tag.w;
        tag_height = tag.h;
        tag_id = tag.id;
        tag_yaw = tag.rot;
        lastTag = tag;
    
        //tagInframe = true;
        targetTagSeen = true;
        lastTargetSeenMs = currentTime;

        // State transitions when we (re)acquire the target
        if (robotState == ROBOT_SEARCHING) {
            StartTagIdBlink(tag.id);
            digitalWrite(LED_PIN, HIGH);
            chassis.SetTwist(0, 0);
            robotState = ROBOT_APPROACHING;
            Serial.println(F("SEARCHING -> APPROACHING (target found)"));
        } 
        else if (robotState == ROBOT_DRIVE_TO_POINT) {
            StartTagIdBlink(tag.id);
            digitalWrite(LED_PIN, HIGH);
            chassis.SetTwist(0, 0);
            robotState = ROBOT_APPROACHING;
            Serial.println(F("DRIVE_TO_POINT -> APPROACHING (target found)"));
        }
    } else {
        tagInframe = false;
    } 

    const unsigned long TAG_RECENT_MS = 200;  // e.g. 0.2s grace period

    if (targetTagSeen && (currentTime - lastTargetSeenMs) < TAG_RECENT_MS) {
        tagInframe = true;
    } else {
        tagInframe = false;
    }

    if (targetTagSeen && robotState == ROBOT_APPROACHING) {
        unsigned long elapsed = currentTime - lastTargetSeenMs;

        if ((elapsed >= TAG_LOST_TIMEOUT_MS) && robotState == ROBOT_APPROACHING) {
            chassis.SetTwist(0.0f, 0.0f);
            robotState = ROBOT_SEARCHING;
            targetTagSeen = false;  
            tagInframe = false;
            Serial.println(F("Target tag lost for 1s -> ROBOT_SEARCHING"));
        }
    }

#ifdef __3D_CAMERA__
/**
 * This will show a tag in the field of view of the camera, though not particularly
 * well. You'll have to zoom out in Teleplot and the scaling won't be proper, but you
 * can see the tag moving.
 * 
 * Contain your excitement.
 */
    char tagStr[80];
    char rotStr[10];

    float r = tag.rot * 3.1416 / 180.0;

    dtostrf(r, 3, 2, rotStr);

    sprintf(tagStr, ">3D|tag3d:S:cube:P:%i:%i:1:R:0:0:%s:W:%i:H:1:D:%i:C:#2ecc71", (tag.cx - 80), (tag.cy - 60), rotStr, tag.w, tag.h);
    Serial.println(tagStr);
#endif

}

void Robot::HandleScanComplete()
{
    chassis.SetTwist(0,0);

    if (tagInframe == true)
    {
        StartTagIdBlink(tag_id);
        robotState = ROBOT_APPROACHING;
        Serial.println("SEARCHING -> APPROACHING");
    }
    else
    {
        robotState = ROBOT_IDLE; 
        Serial.println("NO TAG DETECTED");
        Serial.println("SEARCHING -> IDLE");
    }
}

void Robot::HandleApproachComplete()
{
    chassis.SetTwist(0,0);

    robotState = ROBOT_IDLE;
    Serial.println("APPROACH COMPLETE -> IDLE");
}