#include "user.h"
#include "hardware.h"
#include "driver.h"

void runPreAuton() {}
void runAuton() {}

void runDriver() {
    stopChassis(coast);

    while (true) {
        driverCurvature(controller_1.Axis3.position(), controller_1.Axis1.position());
        wait(20, msec);
    }
}
