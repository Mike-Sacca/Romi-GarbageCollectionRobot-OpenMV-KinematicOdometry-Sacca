// ========== Imports ==========
#include "robot.h"
#include "utils.h"

// ========== Timer Constants ==========
float previous_theta = 0.0f; // global variable to store previous theta value for dead reckoning
const float time_loop = 0.02f; // loop time in seconds (20 ms)

// ========== UpdatePose Constants ==========
const float kRho = 0.5f; 
const float kTheta = 2.5f;
const float maxFwdSpeed = 20.0f; // cm/s
const float maxAngSpeed = 1.5f; // rad/s

// ========== TurnToFace Constants ==========
float theta_kp = 2.0f; // gain value for theta, through testing found to be 3x linear distance kp 
static float MAX_TURN_EFFORT = 1.5f; // in RADIANS
static float ANGLE_TOLERANCE = 0.025f; // in RADIANS

// ========== UPDATE POSE LOGIC ==========

void Robot::UpdatePose(const Twist& twist)
{
    float dTheta = twist.omega * time_loop;

    totalTurn += dTheta;

    currPose.theta = previous_theta + dTheta;

    while (currPose.theta > PI)  currPose.theta -= 2.0f * PI; // use of whiles to regulate theta position over if statements because numbers are dynamic (need constant adjustment for wrap) acceptable in c++
    while (currPose.theta <= -PI) currPose.theta += 2.0f * PI;

    double average_theta = (previous_theta + currPose.theta) / 2.0f; // average theta is previous plus current / 2, discussed in class slides

    currPose.x += twist.u * time_loop * cos(average_theta); // current x / y position found by taking the sin and cosine components of linear velocity

    currPose.y += twist.u * time_loop * sin(average_theta); 

    previous_theta = currPose.theta; // update the previous theta to match current theta 
    
#ifdef __NAV_DEBUG__
    TeleplotPrint("current x", currPose.x);
    TeleplotPrint("current y", currPose.y);
    TeleplotPrint("current theta", currPose.theta);
#endif

}

// ========== SET DESTINATION LOGIC ==========
/**
 * Sets a destination in the lab frame.
 */
void Robot::SetDestination(const Pose& dest)
{
    /**
     * TODO: Turn on LED, as well.
     */
    Serial.print("Setting dest to: ");
    Serial.print(dest.x);
    Serial.print(", ");
    Serial.print(dest.y);
    Serial.print('\n');

    destPose = dest;
    robotState = ROBOT_DRIVE_TO_POINT;
}

// ========== DRIVE TO POINT LOGIC ==========

void Robot::DriveToPoint(void)
{

    if(robotState == ROBOT_DRIVE_TO_POINT) // ensure robot is in correct state before running code
    {

        float dx = destPose.x -  currPose.x; // calculate distance errors in x and y
        float dy = destPose.y -  currPose.y;

        float dest_theta = atan2f(dy, dx); // desired theta is angle to point from current position
        float dist_error = sqrtf(dx*dx + dy*dy); // distance error is linear distance to point

        float raw_delta = dest_theta - currPose.theta; // raw delta is desired theta - current theta
        float heading_error = atan2f(sinf(raw_delta), cosf(raw_delta)); // heading error is wrapped delta angle
        
        float fwdSpeed = kRho * dist_error;      // [cm/s]
        float angSpeed = kTheta * heading_error; // [rad/s]

        if (fwdSpeed >  maxFwdSpeed) fwdSpeed =  maxFwdSpeed;
        if (fwdSpeed < -maxFwdSpeed) fwdSpeed = -maxFwdSpeed;

        if (angSpeed >  maxAngSpeed) angSpeed =  maxAngSpeed;
        if (angSpeed < -maxAngSpeed) angSpeed = -maxAngSpeed;


#ifdef __NAV_DEBUG__
    // Print useful stuff here.
    Serial.print("===============================");
    TeleplotPrint("distanceError", dist_error);
    TeleplotPrint("headingError", heading_error * (180/3.14159));
    TeleplotPrint("X Dist error:", (currPose.x - destPose.x));
    TeleplotPrint("Y Dist Error:", (currPose.y - destPose.y)); 
    TeleplotPrint("Current Theta: ", currPose.theta);
    TeleplotPrint("Dest Theta: ", dest_theta); 
    Serial.print("===============================");

#endif

        chassis.SetTwist(fwdSpeed, angSpeed);
    }
}

// ========== TURN TO ANGLE LOGIC ==========

void Robot::TurnToAngle(void)
{

    if (robotState == ROBOT_TURN_TO_FACE)
    { 
    
    float raw_delta = destPose.theta - currPose.theta;
    float heading_error = atan2f(sinf(raw_delta), cosf(raw_delta));

    float ang_velocity = theta_kp * heading_error;

    if (ang_velocity > MAX_TURN_EFFORT){ang_velocity =  MAX_TURN_EFFORT; } 
    else if (ang_velocity < - MAX_TURN_EFFORT){ang_velocity = - MAX_TURN_EFFORT;}

    chassis.SetTwist(0.0f, ang_velocity);

    } 

}

// ========== CHECKERS ==========

bool Robot::CheckReachedDestination(void)
{
    bool retVal = false; // flag to determine if check has passed, resets automatically on next loop

    float dx = currPose.x - destPose.x; // distance remaining to be traveled is current - desired
    float dy = currPose.y - destPose.y;
    float dist = sqrtf(dx*dx + dy*dy); // linear (radial) distance to point found using pythagoream theorum, plays nicer with romi locomotion than a box check (checking x and y components seperatley)
    const float DIST_TOLERANCE = 2.5f; // 5 cm radial distance tolerance from romi center

    if (dist < DIST_TOLERANCE){ // check if currentl excpected position is within linear tolerance to distination 

        Serial.print("Destination Reached! Check Passed");

        retVal = true; // set flag to true

    }

    return retVal; // return the ending state of flag during this loop 
}

bool Robot::CheckAngleReached(void)
{

    bool retVal = false;

    float raw_delta = destPose.theta - currPose.theta; 
    float heading_error = atan2f(sinf(raw_delta), cosf(raw_delta));

    if (fabsf(heading_error) < ANGLE_TOLERANCE)
    {
        Serial.print("Angle Reached! Check Passed"); 

        retVal = true; 
    }

    return retVal; 
}

// ========== HANDLERS ==========

void Robot::HandleDestination(void)
{
    currPose.x = destPose.x;
    currPose.y = destPose.y;

    robotState = ROBOT_TURN_TO_FACE; // <= NEW CODE 

    // robotState = ROBOT_IDLE <= OLD CODE

    Serial.println("DRIVE_TO_POINT ->  TURN_TO_FACE");
}

void Robot::HandleAngleReached(void){

    robotState = ROBOT_SEARCHING; 

    Serial.println("TURN_TO_FACE ->  SEARCHING");

}