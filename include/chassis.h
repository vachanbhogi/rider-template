#pragma once

#include "hardware.h"

void driveChassis(double left_volts, double right_volts);
void stopChassis(brakeType mode = coast);
void resetChassis();
