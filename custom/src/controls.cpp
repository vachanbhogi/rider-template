#include "controls.h"
#include "hardware.h"
#include "driver.h"
#include "mechs.h"

void runDriver() {
    stopChassis(coast);

    while (true) {
        driverCurvature(controller_1.Axis3.position(), controller_1.Axis1.position());

        if (controller_1.ButtonR1.pressing()) {
            liftUp();
        } else if (controller_1.ButtonR2.pressing()) {
            liftDown();
        } else {
            liftHold();
        }

        wait(10, msec);
    }
}
