#include "odom.h"
#include "chassis.h"

using namespace std;
using namespace vex;

double x_pos = 0.0;
double y_pos = 0.0;
double target_h = 0.0;
double odom_traveled = 0.0;
double odom_speed = 0.0;

static bool odom_started = false;
static bool pose_reset = false;
static bool contact_detected = false;

double vertTrackerIn() {
    if (using_vert_tracker && vert_tracker.installed()) {
        return (vert_tracker.position(deg) / 360.0) * M_PI * vert_tracker_diam;
    }
    return driveInches();
}

double horizTrackerIn() {
    if (using_horiz_tracker && horiz_tracker.installed()) {
        return (horiz_tracker.position(deg) / 360.0) * M_PI * horiz_tracker_diam;
    }
    return 0.0;
}

static int odomLoop() {
    double prev_v = vertTrackerIn();
    double prev_h = horizTrackerIn();
    double prev_deg = imuHeading();
    double prev_t = Brain.timer(msec);

    while (true) {
        if (pose_reset) {
            prev_v = vertTrackerIn();
            prev_h = horizTrackerIn();
            prev_deg = imuHeading();
            pose_reset = false;
        }

        double cur_v = vertTrackerIn();
        double cur_h = horizTrackerIn();
        double cur_deg = imuHeading();
        double now_t = Brain.timer(msec);
        double dt = fmax(0.002, (now_t - prev_t) / 1000.0);
        prev_t = now_t;

        double dv = cur_v - prev_v;
        double dh_in = cur_h - prev_h;
        double d_deg = wrapAngle(cur_deg - prev_deg);

        prev_v = cur_v;
        prev_h = cur_h;
        prev_deg = cur_deg;

        // contact freeze
        double v_pod = fabs(dv / dt);
        double cmd_v = (fabs(left_chassis.voltage(volt)) + fabs(right_chassis.voltage(volt))) * 0.5;

        contact_detected = (cmd_v > 4.0) && (v_pod < 0.15);

        double dh = degToRad(d_deg);
        double prev_rad = degToRad(prev_deg);

        double sinc = (fabs(dh) < 0.01) ? (1.0 - dh * dh / 24.0) : (sin(dh * 0.5) / (dh * 0.5));
        double half_sin = 2.0 * sin(dh * 0.5);

        double x_pod_v = (using_vert_tracker && vert_tracker.installed()) ? vert_tracker_offset : 0.0;
        double y_pod_h = (using_horiz_tracker && horiz_tracker.installed()) ? horiz_tracker_offset : 0.0;

        double ly = sinc * dv + half_sin * x_pod_v;
        double lx = sinc * dh_in - half_sin * y_pod_h;

        double avg_h = prev_rad + dh * 0.5;
        double dx = ly * sin(avg_h) + lx * cos(avg_h);
        double dy = ly * cos(avg_h) - lx * sin(avg_h);

        if (contact_detected) {
            dx = 0.0;
            dy = 0.0;
            odom_speed *= 0.7;
        } else {
            x_pos += dx;
            y_pos += dy;
            double step = hypot(dx, dy);
            odom_traveled += step;
            odom_speed = 0.7 * odom_speed + 0.3 * (step / dt);
        }
        target_h = cur_deg;

        wait(10, msec);
    }
}

void odomInit() {
    resetChassis();
    if (vert_tracker.installed()) {
        vert_tracker.resetPosition();
        vexAbsEncDataRateSet(vert_tracker.index(), 10);
    }
    if (horiz_tracker.installed()) {
        horiz_tracker.resetPosition();
        vexAbsEncDataRateSet(horiz_tracker.index(), 10);
    }
    if (!odom_started) {
        odom_started = true;
        task t(odomLoop);
    }
}

void setPose(double x, double y, double heading_deg) {
    if (imu.installed()) {
        imu.setRotation(heading_deg, deg);
    }
    x_pos = x;
    y_pos = y;
    target_h = heading_deg;
    pose_reset = true;
}

void setHeading(double deg) {
    if (imu.installed()) {
        imu.setRotation(deg, vex::deg);
    }
    target_h = deg;
    pose_reset = true;
}

void setX(double x) {
    x_pos = x;
}

void setY(double y) {
    y_pos = y;
}

bool odomIsContactDetected() {
    return contact_detected;
}

double distTo(double x, double y) {
    return hypot(x - x_pos, y - y_pos);
}

double angleTo(double x, double y) {
    return radToDeg(atan2(x - x_pos, y - y_pos));
}

bool isNear(double x, double y, double radius_in) {
    return distTo(x, y) <= radius_in;
}

