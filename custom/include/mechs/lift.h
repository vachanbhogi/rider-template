#pragma once

#include "hardware.h"

// lift control
void liftInit();
void liftUp(double volts = 12.0);
void liftDown(double volts = 12.0);
void liftHold();
