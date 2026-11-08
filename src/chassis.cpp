#include "chassis.h"

using namespace std;
using namespace vex;

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

double degToRad(double deg) {
    return deg * M_PI / 180.0;
}

double radToDeg(double rad) {
    return rad * 180.0 / M_PI;
}

double wrapAngle(double deg) {
    while (deg >= 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}

double leftDeg() {
    return left_chassis.position(deg);
}

double rightDeg() {
    return right_chassis.position(deg);
}

double driveInches() {
    return ((leftDeg() + rightDeg()) * 0.5 / 360.0) * wheel_circ;
}

double imuHeading() {
    if (!imu.installed()) return 0.0;
    return imu.rotation(deg);
}

void clampMin(double& l, double& r, double min_v) {
    double mag = fmax(fabs(l), fabs(r));
    if (mag > 1e-6 && mag < min_v) {
        double s = min_v / mag;
        l *= s;
        r *= s;
    }
}

void clampMax(double& l, double& r, double max_v) {
    double mag = fmax(fabs(l), fabs(r));
    if (mag > max_v && max_v > 0.0) {
        double s = max_v / mag;
        l *= s;
        r *= s;
    }
}
