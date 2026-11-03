#include "chassis.h"

void driveChassis(double left_volts, double right_volts) {
    left_chassis.spin(fwd, left_volts, volt);
    right_chassis.spin(fwd, right_volts, volt);
}

void stopChassis(brakeType mode) {
    left_chassis.stop(mode);
    right_chassis.stop(mode);
}

void resetChassis() {
    left_chassis.resetPosition();
    right_chassis.resetPosition();
}
