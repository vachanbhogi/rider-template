#pragma once

#include "vex.h"

using namespace vex;

extern brain Brain;
extern controller controller_1;

extern motor left_1;
extern motor left_2;
extern motor left_3;
extern motor_group left_chassis;

extern motor right_1;
extern motor right_2;
extern motor right_3;
extern motor_group right_chassis;

extern inertial imu;

// lift motors
extern motor lift_left;
extern motor lift_right;
extern motor_group lift_motors;
