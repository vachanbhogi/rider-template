#include "mechs/lift.h"

void liftInit() {
    lift_motors.setStopping(hold);
    lift_motors.resetPosition();
}

void liftUp(double volts) {
    lift_motors.spin(fwd, volts, volt);
}

void liftDown(double volts) {
    lift_motors.spin(reverse, volts, volt);
}

void liftHold() {
    lift_motors.stop(hold);
}
