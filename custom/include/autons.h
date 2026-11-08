#pragma once

#include "nav.h"
#include "odom.h"
#include "chassis.h"
#include "mechs.h"

// auton routines
void forward24();

// auton lifecycle
extern int current_auton_selection;
extern void (*autons[])();
extern const int auton_count;
void runPreAuton();
void runAuton();
