#include "hardware.h"

brain Brain;
controller controller_1;

// drivetrain
motor left_1(PORT1, ratio6_1, true);
motor left_2(PORT2, ratio6_1, true);
motor left_3(PORT3, ratio6_1, true);
motor_group left_chassis(left_1, left_2, left_3);

motor right_1(PORT4, ratio6_1, false);
motor right_2(PORT5, ratio6_1, false);
motor right_3(PORT6, ratio6_1, false);
motor_group right_chassis(right_1, right_2, right_3);

// sensors
inertial imu(PORT7);
rotation horiz_tracker(PORT8, false);
rotation vert_tracker(PORT9, false);

// example mech
motor lift_left(PORT10, ratio18_1, true);
motor lift_right(PORT11, ratio18_1, false);
motor_group lift_motors(lift_left, lift_right);

// drivetrain & tracker geometry
double track_width = 12.0;
double wheel_diam = 2.75;
double drive_gear_ratio = 1.0;
double wheel_circ = drive_gear_ratio * wheel_diam * M_PI;

bool using_horiz_tracker = false;
bool using_vert_tracker = false;
double horiz_tracker_diam = 2.0;
double horiz_tracker_offset = 0.0;
double vert_tracker_diam = 2.0;
double vert_tracker_offset = 0.0;
