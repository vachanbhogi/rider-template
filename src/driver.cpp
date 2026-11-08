#include "driver.h"
#include <math.h>

static double curveStick(double val, double cubic, double deadband) {
    double a = fabs(val);
    if (a <= deadband) return 0.0;

    double norm = (a - deadband) / (100.0 - deadband);
    double curve = (cubic * norm * norm * norm) + ((1.0 - cubic) * norm);

    return (val > 0.0) ? curve * 12.0 : -curve * 12.0;
}

static void output(double l, double r) {
    double mag = fmax(fabs(l), fabs(r));
    if (mag > 12.0) {
        l = (l / mag) * 12.0;
        r = (r / mag) * 12.0;
    }
    driveChassis(l, r);
}

void driverCurvature(double throttle_stick, double steer_stick, double curve, double deadband) {
    double throttle = curveStick(throttle_stick, curve, deadband);
    double turn = curveStick(steer_stick, curve * 0.3, deadband);

    // continuous quick turn blend
    double quick_blend = 1.0 - fmin(1.0, fabs(throttle) / 1.5);
    double quick_l = 0.5 * turn;
    double quick_r = -0.5 * turn;

    // curvature steering
    double curve_gain = 1.30;
    double eff = fmax(fabs(throttle), 12.0 * 0.20);
    double curve_l = throttle + eff * (turn / 12.0) * curve_gain;
    double curve_r = throttle - eff * (turn / 12.0) * curve_gain;

    output((quick_blend * quick_l) + ((1.0 - quick_blend) * curve_l),
           (quick_blend * quick_r) + ((1.0 - quick_blend) * curve_r));
}

void driverArcade(double throttle_stick, double steer_stick, double curve, double deadband) {
    double throttle = curveStick(throttle_stick, curve, deadband);
    double turn = curveStick(steer_stick, curve * 0.4, deadband);
    output(throttle + turn, throttle - turn);
}

void driverTank(double left_stick, double right_stick, double curve, double deadband) {
    output(curveStick(left_stick, curve, deadband), curveStick(right_stick, curve, deadband));
}

bool clicked(const controller::button& b) {
    static const controller::button* btns[16];
    static bool prev[16] = {false};
    static int count = 0;

    int idx = -1;
    for (int i = 0; i < count; i++) {
        if (btns[i] == &b) { idx = i; break; }
    }
    if (idx == -1 && count < 16) {
        idx = count++;
        btns[idx] = &b;
    }

    bool state = b.pressing();
    bool edge = state && !prev[idx];
    prev[idx] = state;
    return edge;
}

