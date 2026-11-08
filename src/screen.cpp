#include "screen.h"
#include "odom.h"
#include "chassis.h"

using namespace std;
using namespace vex;

static bool screen_started = false;

static int screenLoop() {
    while (true) {
        Brain.Screen.clearScreen(black);
        Brain.Screen.setPenColor(white);
        Brain.Screen.setFillColor(black);
        Brain.Screen.setFont(mono40);

        Brain.Screen.printAt(40, 60, false, "X:     %.2f", x_pos);
        Brain.Screen.printAt(40, 120, false, "Y:     %.2f", y_pos);
        Brain.Screen.printAt(40, 180, false, "Theta: %.1f", imuHeading());

        wait(50, msec);
    }
    return 0;
}

void screenInit() {
    if (!screen_started) {
        screen_started = true;
        task t(screenLoop);
    }
}
