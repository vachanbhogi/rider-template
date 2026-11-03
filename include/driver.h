#pragma once

#include "chassis.h"

// default curve (cubic): 0.50, deadband: 4.0%
void driverCurvature(double throttle_stick, double steer_stick, double curve = 0.50, double deadband = 4.0);
void driverArcade(double throttle_stick, double steer_stick, double curve = 0.50, double deadband = 4.0);
void driverTank(double left_stick, double right_stick, double curve = 0.50, double deadband = 4.0);
