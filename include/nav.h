#pragma once

#include "hardware.h"

void driveTo(double dist_in, double timeout_ms = 1200, double max_v = 10.0,
             double heading_deg = 999.0, bool (*early_stop)() = nullptr, double min_v = 0.0);

void moveToPoint(double x, double y, int dir = 1, double timeout_ms = 1600, double max_v = 10.0);

void curveTo(double x, double y, int dir = 1, double timeout_ms = 1500, double max_v = 10.0,
             bool (*early_stop)() = nullptr);

// side: -1 left, +1 right
void arcTo(double heading_deg, double radius_in = 0.0, int side = -1,
           double timeout_ms = 1200, double max_v = 9.0);

void curveToPose(double x, double y, double final_deg, double offset = 0.0,
                 int dir = 1, double timeout_ms = 1800, double max_v = 10.0,
                 bool (*early_stop)() = nullptr);

void reverseTo(double x, double y, double timeout_ms = 1000, double max_v = 7.5,
               double hold_heading_deg = 999.0);

void turnTo(double heading_deg, double timeout_ms = 900, double max_v = 9.0);
void turnToPoint(double x, double y, int dir = 1, double timeout_ms = 900, double max_v = 9.0);

void seatWall(double volts = -8.0, double timeout_ms = 450, double expected_h = 0.0);
