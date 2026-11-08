#pragma once

#include "hardware.h"

extern double x_pos;
extern double y_pos;
extern double target_h;
extern double odom_traveled;
extern double odom_speed;

void odomInit();
void setPose(double x, double y, double heading_deg);
void setHeading(double deg);
void setX(double x);
void setY(double y);

double vertTrackerIn();
double horizTrackerIn();
bool odomIsContactDetected();

double distTo(double x, double y);
double angleTo(double x, double y);
bool isNear(double x, double y, double radius_in);
