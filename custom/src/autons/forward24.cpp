#include "autons.h"

void forward24() {
    setPose(0.0, 0.0, 0.0);
    driveTo(24.0, 1500, 10.0);
}
