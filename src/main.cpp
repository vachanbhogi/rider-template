#include "controls.h"
#include "autons.h"

using namespace vex;

competition Competition;

int main() {
    Competition.autonomous(runAuton);
    Competition.drivercontrol(runDriver);
    runPreAuton();

    while (true) {
        wait(100, msec);
    }
}
