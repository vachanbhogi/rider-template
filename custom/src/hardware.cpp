#include "hardware.h"

brain Brain;
controller controller_1;

// 600 rpm blue drive
motor left_1(PORT1, ratio6_1, true);
motor left_2(PORT2, ratio6_1, true);
motor left_3(PORT3, ratio6_1, true);
motor_group left_chassis(left_1, left_2, left_3);

motor right_1(PORT4, ratio6_1, false);
motor right_2(PORT5, ratio6_1, false);
motor right_3(PORT6, ratio6_1, false);
motor_group right_chassis(right_1, right_2, right_3);

inertial imu(PORT7);

// example mech - lift with 2 motors facing each other
motor lift_left(PORT18, ratio18_1, true);
motor lift_right(PORT19, ratio18_1, false);
motor_group lift_motors(lift_left, lift_right);
