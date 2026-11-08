#pragma once

#include "hardware.h"

void driveChassis(double left_volts, double right_volts);
void stopChassis(brakeType mode = coast);
void resetChassis();

double degToRad(double deg);
double radToDeg(double rad);
double wrapAngle(double deg);

double leftDeg();
double rightDeg();
double driveInches();
double imuHeading();

void clampMin(double& l, double& r, double min_v);
void clampMax(double& l, double& r, double max_v);
