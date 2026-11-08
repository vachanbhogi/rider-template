#include "autons.h"
#include "hardware.h"
#include "odom.h"
#include "screen.h"
#include "mechs.h"

using namespace std;
using namespace vex;

void runPreAuton() {
    if (imu.installed()) {
        imu.calibrate();
        while (imu.isCalibrating()) {
            wait(10, msec);
        }
    }
    mechsInit();
    odomInit();
    screenInit();
}

int current_auton_selection = 0;

void (*autons[])() = {
    forward24
};

const int auton_count = sizeof(autons) / sizeof(autons[0]);

void runAuton() {
    if (current_auton_selection >= 0 && current_auton_selection < auton_count) {
        autons[current_auton_selection]();
    }
}
