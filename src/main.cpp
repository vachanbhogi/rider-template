#include "vex.h"

using namespace vex;

competition Competition;

void pre_auton() {
}

void autonomous() {
}

void usercontrol() {
    while (true) {
        wait(20, msec);
    }
}

int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
    pre_auton();

    while (true) {
        wait(100, msec);
    }
}
