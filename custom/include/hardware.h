#pragma once

#include "vex.h"

using namespace vex;

extern brain Brain;
extern controller controller_1;

// drivetrain motors
extern motor left_1;
extern motor left_2;
extern motor left_3;
extern motor_group left_chassis;

extern motor right_1;
extern motor right_2;
extern motor right_3;
extern motor_group right_chassis;

// sensors
extern inertial imu;
extern rotation horiz_tracker;
extern rotation vert_tracker;

// example mech - lift
extern motor lift_left;
extern motor lift_right;
extern motor_group lift_motors;

// drivetrain & odom
extern double track_width;
extern double wheel_diam;
extern double drive_gear_ratio;
extern double wheel_circ;

extern bool using_horiz_tracker;
extern bool using_vert_tracker;
extern double horiz_tracker_diam;
extern double horiz_tracker_offset;
extern double vert_tracker_diam;
extern double vert_tracker_offset;
