#pragma once
#include <Arduino.h>

void TeleplotPrint(const char* var, float value);
void TeleplotPrintXY(const char* topic, float x, float y);

/**
 * Pose includes information about the 2D pose of a robot: x, y, and heading.
 */
struct Pose
{
    float x = 0;
    float y = 0;
    float theta = 0;

    Pose(void) {}
    Pose(float x_, float y_, float th_) : x(x_), y(y_), theta(th_) {}
};

/**
 * Twist is similar to Pose, but velocities, instead of positions.
 * 
 * Whereas Pose is position/heading, Twist contains velocity/ang. vel.
 */
struct Twist
{
    float u = 0;
    float v = 0; // This will always be 0 in the robot frame.
    float omega = 0;

    Twist(void) {}
    Twist(float u_, float v_, float om_) : u(u_), v(v_), omega(om_) {}
};
