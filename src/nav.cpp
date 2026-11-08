#include "nav.h"
#include "chassis.h"
#include "odom.h"
#include "gains.h"
#include <cmath>

using namespace std;
using namespace vex;

void driveTo(double dist_in, double timeout_ms, double max_v,
             double heading_deg, bool (*early_stop)(), double min_v) {
    double t0 = Brain.timer(msec);
    double start_in = vertTrackerIn();
    double target_abs = fabs(dist_in);
    double dir = (dist_in >= 0.0) ? 1.0 : -1.0;
    double init_h = (heading_deg < 900.0) ? heading_deg : imuHeading();

    double prev_err = target_abs;
    double prev_h_err = 0.0;
    double cmd_v = (min_v > 0.0) ? min_v : 0.0;
    bool first = true;
    int settled_cnt = 0;

    double moved_in = start_in;
    double moved_ms = t0;

    while (Brain.timer(msec) - t0 < timeout_ms) {
        if (early_stop != nullptr && early_stop()) break;

        double cur_in = vertTrackerIn();
        double moved = (cur_in - start_in) * dir;
        double err = target_abs - moved;

        if (min_v > 0.0 && err <= 1.2) break;

        if (fabs(err) < 0.5) {
            if (++settled_cnt * 10 >= 40) break;
        } else {
            settled_cnt = 0;
        }

        // stall check
        if (fabs(cur_in - moved_in) > 0.08) {
            moved_in = cur_in;
            moved_ms = Brain.timer(msec);
        } else if (fabs(moved) > 1.5 && (Brain.timer(msec) - moved_ms > 150.0)) {
            break;
        }

        double d_err = first ? 0.0 : (err - prev_err);
        prev_err = err;

        double eff_max = max_v;
        if (early_stop != nullptr && err < 8.5 && err > 0.0) {
            double u = fmax(0.0, fmin(1.0, (err - 2.0) / 6.5));
            eff_max = fmin(max_v, 4.5 + (max_v - 4.5) * u);
        }

        double raw_v = DRIVE_KP * err + DRIVE_KD * d_err;
        raw_v = fmax(-eff_max, fmin(eff_max, raw_v));
        if (min_v > 0.0 && raw_v < min_v) {
            raw_v = min_v;
        } else if (fabs(err) >= 0.35 && fabs(raw_v) < DRIVE_KS) {
            raw_v = copysign(DRIVE_KS, err);
        }

        // slew
        double max_slew = 0.8;
        if (fabs(raw_v) > fabs(cmd_v) + max_slew) {
            cmd_v += copysign(max_slew, raw_v);
        } else {
            cmd_v = raw_v;
        }

        double out = cmd_v * dir;

        double h_err = wrapAngle(init_h - imuHeading());
        double dh_err = first ? 0.0 : wrapAngle(h_err - prev_h_err);
        first = false;
        prev_h_err = h_err;

        double steer = HEADING_KP * h_err + HEADING_KD * dh_err;

        double l = out + steer;
        double r = out - steer;
        clampMax(l, r, max_v);
        driveChassis(l, r);
        wait(10, msec);
    }

    if (min_v <= 0.0) {
        stopChassis(hold);
    }
    target_h = init_h;
}

void moveToPoint(double x, double y, int dir, double timeout_ms, double max_v) {
    double dx = x - x_pos;
    double dy = y - y_pos;
    double dist = hypot(dx, dy);
    if (dist < 0.5) return;

    double t0 = Brain.timer(msec);
    turnToPoint(x, y, dir, fmin(timeout_ms * 0.45, 800.0), max_v);

    double elapsed = Brain.timer(msec) - t0;
    double rem_time = timeout_ms - elapsed;
    if (rem_time > 150.0) {
        dx = x - x_pos;
        dy = y - y_pos;
        dist = hypot(dx, dy);
        double drive_dir = (dir >= 0) ? 1.0 : -1.0;
        driveTo(dist * drive_dir, rem_time, max_v);
    }
}

void curveTo(double x, double y, int dir, double timeout_ms, double max_v, bool (*early_stop)()) {
    double t0 = Brain.timer(msec);
    double sign_dir = (dir >= 0) ? 1.0 : -1.0;
    double prev_dist = hypot(x - x_pos, y - y_pos);
    double prev_h_err = 0.0;
    double cur_cmd_v = 0.0;
    bool first = true;
    int settled_cnt = 0;

    double init_dx = x - x_pos;
    double init_dy = y - y_pos;
    double init_len = hypot(init_dx, init_dy);
    double unit_x = (init_len > 0.01) ? (init_dx / init_len) : 0.0;
    double unit_y = (init_len > 0.01) ? (init_dy / init_len) : 1.0;
    double frozen_target_h = radToDeg(atan2(init_dx, init_dy));
    if (sign_dir < 0.0) frozen_target_h = wrapAngle(frozen_target_h + 180.0);

    double moved_in = vertTrackerIn();
    double moved_ms = t0;

    while (Brain.timer(msec) - t0 < timeout_ms) {
        if (early_stop != nullptr && early_stop()) break;

        double dx = x - x_pos;
        double dy = y - y_pos;
        double dist = hypot(dx, dy);

                double along = (dx * unit_x + dy * unit_y) * sign_dir;

        if (dist < 0.65) {
            if (++settled_cnt * 10 >= 40) break;
        } else {
            settled_cnt = 0;
        }

        if (dist < 3.0 && along <= 0.0) break;

        // stall check
        double cur_in = vertTrackerIn();
        if (fabs(cur_in - moved_in) > 0.08) {
            moved_in = cur_in;
            moved_ms = Brain.timer(msec);
        } else if (dist < 6.0 && (Brain.timer(msec) - t0 > 250.0) && (Brain.timer(msec) - moved_ms > 150.0)) {
            break;
        }

        double target_h;
        if (dist > 2.5) {
            target_h = radToDeg(atan2(dx, dy));
            if (sign_dir < 0.0) target_h = wrapAngle(target_h + 180.0);
            frozen_target_h = target_h;
        } else {
            target_h = frozen_target_h;
        }

        double h_err = wrapAngle(target_h - imuHeading());
        double cos_scale = fmax(0.0, cos(degToRad(h_err)));
        cos_scale = (cos_scale * cos_scale) / (1.0 + DRIFT_K * fabs(h_err));

        double d_dist = first ? 0.0 : (dist - prev_dist);
        prev_dist = dist;

        double desired_v = (BOOMERANG_KP_DIST * dist + BOOMERANG_KD_DIST * d_dist) * cos_scale;
        desired_v = fmax(-max_v, fmin(max_v, desired_v));

        if (dist >= 0.5 && fabs(h_err) < 40.0 && fabs(desired_v) < DRIVE_KS) {
            desired_v = DRIVE_KS;
        }

        // slew
        double max_slew = 0.8;
        if (desired_v > cur_cmd_v + max_slew) cur_cmd_v += max_slew;
        else if (desired_v < cur_cmd_v - max_slew) cur_cmd_v -= max_slew;
        else cur_cmd_v = desired_v;

        double drive_v = cur_cmd_v * sign_dir;

        double dh_err = first ? 0.0 : wrapAngle(h_err - prev_h_err);
        first = false;
        prev_h_err = h_err;

        double steer_v = BOOMERANG_KP_TURN * h_err + BOOMERANG_KD_TURN * dh_err;
        double max_steer = fmax(3.5, max_v * 0.7);
        steer_v = fmax(-max_steer, fmin(max_steer, steer_v));

        double l = drive_v + steer_v;
        double r = drive_v - steer_v;
        clampMax(l, r, max_v);
        driveChassis(l, r);
        wait(10, msec);
    }

    stopChassis(hold);
    target_h = imuHeading();
}

void arcTo(double heading_deg, double radius_in, int side, double timeout_ms, double max_v) {
    double t0 = Brain.timer(msec);
    double prev_err = 0.0;
    bool first = true;
    int settled_cnt = 0;

    bool is_locked_pivot = (radius_in <= 0.05);
    double r_ratio = 0.0;
    if (!is_locked_pivot) {
        r_ratio = radius_in / (radius_in + track_width);
    }

    if (is_locked_pivot) {
        if (side < 0) left_chassis.stop(hold);
        else right_chassis.stop(hold);
    }

    while (Brain.timer(msec) - t0 < timeout_ms) {
        double err = wrapAngle(heading_deg - imuHeading());
        double d_err = first ? 0.0 : wrapAngle(err - prev_err);
        first = false;
        prev_err = err;

        if (fabs(err) < TURN_SETTLE_DEG && fabs(d_err) < 0.35) {
            if (++settled_cnt * 10 >= TURN_SETTLE_MS) break;
        } else {
            settled_cnt = 0;
        }

        if (is_locked_pivot) {
            // locked swing
            double out = (TURN_KP * 1.6) * err + TURN_KD * d_err;
            out = fmax(-max_v, fmin(max_v, out));
            if (fabs(err) >= TURN_SETTLE_DEG && fabs(out) < TURN_KS + 0.5) {
                out = copysign(TURN_KS + 0.5, err);
            }

            if (side < 0) {
                left_chassis.stop(hold);
                right_chassis.spin(fwd, -out, volt);
            } else {
                left_chassis.spin(fwd, out, volt);
                right_chassis.stop(hold);
            }
        } else {
            // arc ratio
            double v_outer = fmax(0.0, fmin(max_v, (TURN_KP * 1.8) * fabs(err) + TURN_KD * fabs(d_err)));
            if (fabs(err) >= TURN_SETTLE_DEG && v_outer < TURN_KS + 1.0) {
                v_outer = TURN_KS + 1.0;
            }
            double v_inner = v_outer * r_ratio;

            if (side < 0) {
                double l = v_inner;
                double r = v_outer;
                clampMax(l, r, max_v);
                driveChassis(l, r);
            } else {
                double l = v_outer;
                double r = v_inner;
                clampMax(l, r, max_v);
                driveChassis(l, r);
            }
        }

        wait(10, msec);
    }

    stopChassis(hold);
    target_h = heading_deg;
}

void curveToPose(double x, double y, double final_deg, double offset,
                    int dir, double timeout_ms, double max_v, bool (*early_stop)()) {
    double t0 = Brain.timer(msec);
    double sign_dir = (dir >= 0) ? 1.0 : -1.0;
    double final_rad = degToRad(final_deg);

    double stop_x = x - offset * sin(final_rad) * sign_dir;
    double stop_y = y - offset * cos(final_rad) * sign_dir;

    double lookahead = 9.0;
    double blend_dist = 6.5;

    double prev_dist = hypot(stop_x - x_pos, stop_y - y_pos);
    double prev_h_err = 0.0;
    bool first = true;
    int settled_cnt = 0;

    double moved_in = vertTrackerIn();
    double moved_ms = t0;

    while (Brain.timer(msec) - t0 < timeout_ms) {
        if (early_stop != nullptr && early_stop()) break;

        double dx = stop_x - x_pos;
        double dy = stop_y - y_pos;
        double dist = hypot(dx, dy);

                double along = (dx * sin(final_rad) + dy * cos(final_rad)) * sign_dir;

        double cur_h_rad = degToRad(imuHeading());
        double target_dx = x - x_pos;
        double target_dy = y - y_pos;
        double actual_remaining = (target_dx * sin(cur_h_rad) + target_dy * cos(cur_h_rad)) * sign_dir - offset;

        if (dist < 0.65 || (offset > 0.5 && actual_remaining <= 0.25 && dist < 3.0)) {
            if (++settled_cnt * 10 >= 30) break;
        } else {
            settled_cnt = 0;
        }

        if ((dist < 2.0 && along <= 0.0) || (offset > 0.5 && actual_remaining <= 0.0 && dist < 2.5)) break;

        // stall check
        double cur_in = vertTrackerIn();
        if (fabs(cur_in - moved_in) > 0.1) {
            moved_in = cur_in;
            moved_ms = Brain.timer(msec);
        } else if (dist < 6.0 && (Brain.timer(msec) - t0 > 250.0) && (Brain.timer(msec) - moved_ms > 150.0)) {
            break;
        }

        double carrot_along = fmax(0.0, along - lookahead);
        double carrot_x = stop_x - carrot_along * sin(final_rad) * sign_dir;
        double carrot_y = stop_y - carrot_along * cos(final_rad) * sign_dir;

        double c_dx = carrot_x - x_pos;
        double c_dy = carrot_y - y_pos;
        double raw_aim = (hypot(c_dx, c_dy) > 0.25) ? radToDeg(atan2(c_dx, c_dy)) : final_deg;
        if (sign_dir < 0.0 && hypot(c_dx, c_dy) > 0.25) raw_aim = wrapAngle(raw_aim + 180.0);

        // heading blend
        double aim_deg = raw_aim;
        if (dist < blend_dist) {
            double u = fmax(0.0, dist / blend_dist);
            u = 0.5 * (1.0 - cos(M_PI * u));
            double err_aim = wrapAngle(raw_aim - final_deg);
            aim_deg = final_deg + err_aim * u;
        }

                double h_err = wrapAngle(aim_deg - imuHeading());
        double cos_scale = fmax(0.0, cos(degToRad(h_err)));
        cos_scale = (cos_scale * cos_scale) / (1.0 + 0.015 * fabs(h_err));

        double d_dist = first ? 0.0 : (dist - prev_dist);
        prev_dist = dist;

        double eff_max = max_v;
        if (early_stop != nullptr && dist < 8.5 && dist > 0.0) {
            double u = fmax(0.0, fmin(1.0, (dist - 2.0) / 6.5));
            eff_max = fmin(max_v, 4.5 + (max_v - 4.5) * u);
        }

        double drive_v = (BOOMERANG_KP_DIST * dist + BOOMERANG_KD_DIST * d_dist) * cos_scale * sign_dir;
        drive_v = fmax(-eff_max, fmin(eff_max, drive_v));
        if (dist >= 0.35 && fabs(h_err) < 60.0 && fabs(drive_v) < DRIVE_KS + 1.0) {
            drive_v = copysign(DRIVE_KS + 1.0, sign_dir);
        }

        double dh_err = first ? 0.0 : wrapAngle(h_err - prev_h_err);
        first = false;
        prev_h_err = h_err;

        double steer_v = BOOMERANG_KP_TURN * h_err + BOOMERANG_KD_TURN * dh_err;
        if (fabs(h_err) > 25.0 && fabs(steer_v) < TURN_KS + 0.8) {
            steer_v = copysign(TURN_KS + 0.8, h_err);
        }
        double max_steer = (fabs(h_err) > 25.0) ? fmax(5.0, max_v * 0.85) : fmax(3.5, max_v * 0.65);
        steer_v = fmax(-max_steer, fmin(max_steer, steer_v));

        double l = drive_v + steer_v;
        double r = drive_v - steer_v;
        clampMax(l, r, max_v);
        driveChassis(l, r);
        wait(10, msec);
    }

    stopChassis(hold);
    target_h = final_deg;
}

void reverseTo(double x, double y, double timeout_ms, double max_v, double hold_heading_deg) {
    double t0 = Brain.timer(msec);
    double h_lock = (hold_heading_deg < 900.0) ? hold_heading_deg : imuHeading();
    double h_rad = degToRad(h_lock);

    double cur_cmd_v = 0.0;
    int settled_cnt = 0;

    double moved_in = vertTrackerIn();
    double moved_ms = t0;

    while (Brain.timer(msec) - t0 < timeout_ms) {
                double rem = (x_pos - x) * sin(h_rad) + (y_pos - y) * cos(h_rad);

                if (rem <= 0.25 || hypot(x - x_pos, y - y_pos) < 0.65) {
            if (++settled_cnt * 10 >= 40) break;
        } else {
            settled_cnt = 0;
        }

        // stall check
        double cur_in = vertTrackerIn();
        if (fabs(cur_in - moved_in) > 0.08) {
            moved_in = cur_in;
            moved_ms = Brain.timer(msec);
        } else if (Brain.timer(msec) - moved_ms > 150.0 && Brain.timer(msec) - t0 > 200.0) {
            break;
        }

        double h_err = wrapAngle(h_lock - imuHeading());

        // cross-track
        double cross = (x_pos - x) * cos(h_rad) - (y_pos - y) * sin(h_rad);
        double steer_corr = fmax(-8.0, fmin(8.0, cross * 2.0));
        double steer_target_err = wrapAngle(h_err + steer_corr);

        double desired_v = fmax(0.0, fmin(max_v, BOOMERANG_KP_DIST * rem));
        if (rem > 0.5 && desired_v < DRIVE_KS) desired_v = DRIVE_KS;

        // slew
        double max_slew = 0.8;
        if (desired_v > cur_cmd_v + max_slew) cur_cmd_v += max_slew;
        else if (desired_v < cur_cmd_v - max_slew) cur_cmd_v -= max_slew;
        else cur_cmd_v = desired_v;

        double drive_v = -cur_cmd_v;

        double steer_v = HEADING_KP * steer_target_err;
        double max_steer = fmax(3.0, max_v * 0.45);
        steer_v = fmax(-max_steer, fmin(max_steer, steer_v));

        double l = drive_v + steer_v;
        double r = drive_v - steer_v;
        clampMax(l, r, max_v);
        driveChassis(l, r);

        wait(10, msec);
    }
    stopChassis(hold);
}

void turnTo(double heading_deg, double timeout_ms, double max_v) {
    double t0 = Brain.timer(msec);
    double prev_err = 0.0;
    bool first = true;
    int settled_cnt = 0;

    while (Brain.timer(msec) - t0 < timeout_ms) {
        double err = wrapAngle(heading_deg - imuHeading());
        double d_err = first ? 0.0 : wrapAngle(err - prev_err);
        first = false;
        prev_err = err;

        if (fabs(err) < TURN_SETTLE_DEG && fabs(d_err) < 0.35) {
            if (++settled_cnt * 10 >= TURN_SETTLE_MS) break;
        } else {
            settled_cnt = 0;
        }

        double out = TURN_KP * err + TURN_KD * d_err;
        out = fmax(-max_v, fmin(max_v, out));
        if (fabs(err) >= TURN_SETTLE_DEG && fabs(out) < TURN_KS) {
            out = copysign(TURN_KS, err);
        }

        driveChassis(out, -out);
        wait(10, msec);
    }

    stopChassis(hold);
    target_h = heading_deg;
}

void turnToPoint(double x, double y, int dir, double timeout_ms, double max_v) {
    double t0 = Brain.timer(msec);
    double prev_err = 0.0;
    bool first = true;
    int settled_cnt = 0;

    while (Brain.timer(msec) - t0 < timeout_ms) {
        double dx = x - x_pos;
        double dy = y - y_pos;
        if (hypot(dx, dy) < 0.25) break;

        double target_deg = radToDeg(atan2(dx, dy));
        if (dir < 0) target_deg = wrapAngle(target_deg + 180.0);

        double err = wrapAngle(target_deg - imuHeading());
        double d_err = first ? 0.0 : wrapAngle(err - prev_err);
        first = false;
        prev_err = err;

        if (fabs(err) < TURN_SETTLE_DEG && fabs(d_err) < 0.35) {
            if (++settled_cnt * 10 >= TURN_SETTLE_MS) break;
        } else {
            settled_cnt = 0;
        }

        double out = TURN_KP * err + TURN_KD * d_err;
        out = fmax(-max_v, fmin(max_v, out));
        if (fabs(err) >= TURN_SETTLE_DEG && fabs(out) < TURN_KS) {
            out = copysign(TURN_KS, err);
        }

        driveChassis(out, -out);
        wait(10, msec);
    }

    stopChassis(hold);
    double dx = x - x_pos;
    double dy = y - y_pos;
    double final_deg = radToDeg(atan2(dx, dy));
    if (dir < 0) final_deg = wrapAngle(final_deg + 180.0);
    target_h = final_deg;
}

void seatWall(double volts, double timeout_ms, double expected_h) {
    double t0 = Brain.timer(msec);
    double still_t0 = -1.0;

    while (Brain.timer(msec) - t0 < timeout_ms) {
        driveChassis(volts, volts);
        double now = Brain.timer(msec);
        if (now - t0 > 80.0 && odom_speed < 1.0) {
            if (still_t0 < 0.0) still_t0 = now;
            else if (now - still_t0 >= 40.0) break;
        } else {
            still_t0 = -1.0;
        }
        wait(10, msec);
    }

    stopChassis(hold);
    setHeading(expected_h);
}
