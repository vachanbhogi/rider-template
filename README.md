# Rider

[![VEX V5](https://img.shields.io/badge/VEX-V5-red.svg)](https://www.vexrobotics.com/v5)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Release](https://img.shields.io/github/v/release/vachanbhogi/rider-template)](https://github.com/vachanbhogi/rider-template/releases)

A fast, lightweight C++ template for VEX V5 robots.

All your robot code (ports, driver controls, autons, mechs) lives in `custom/`. The core motion code and odometry live in `include/` and `src/` so you can pull template updates without breaking your robot code.

Created by **Vachan Bhogi (2715N)**.

---

## Features

- **Coordinates**: `moveToPoint(x, y, timeout)` and `turnTo(deg, timeout)` in inches and degrees (0° = north, clockwise positive).
- **Odometry**: Sinc-based tracking (`sinc(Δθ / 2)`) for 0, 1, or 2 tracking wheels + inertial sensor.
- **Anti-slip**: Pauses tracking wheel integration during hard pushes or goal collisions so wheel scrub doesn't mess up your pose.
- **Slew limiter**: Voltage slew rate to prevent tripping the 20A brain limiter under sudden direction changes.
- **Driver modes**: Built-in split arcade, tank, and curvature (cheesy drive) with adjustable cubic curves and deadband.

---

## Quickstart

1. Download the latest `Rider.zip` from [Releases](https://github.com/vachanbhogi/rider-template/releases) and open the folder in VS Code (with the VEX extension).
2. Set your motor ports, cartridge, and track width in `custom/src/hardware.cpp`:
   ```cpp
   motor l1(PORT1, ratio6_1, true);
   motor l2(PORT2, ratio6_1, true);
   motor l3(PORT3, ratio6_1, true);

   motor r1(PORT4, ratio6_1, false);
   motor r2(PORT5, ratio6_1, false);
   motor r3(PORT6, ratio6_1, false);

   motor_group left_motors(l1, l2, l3);
   motor_group right_motors(r1, r2, r3);

   double track_width = 12.5;
   double wheel_diameter = 2.75;
   double drive_gear_ratio = 36.0 / 48.0;
   ```
   If you don't have tracking wheels, just set:
   ```cpp
   bool using_vert_tracker = false;
   bool using_horiz_tracker = false;
   ```
   Rider will use your drive motor encoders and IMU instead.
3. Build and download (`Ctrl+Shift+B` or `Cmd+Shift+B`).

---

## File Structure

Only edit `custom/`. Everything else is the engine.

```
rider-template/
├── custom/                 # your code
│   ├── include/
│   │   ├── autons.h        # auton declarations
│   │   ├── controls.h      # driver controls
│   │   ├── gains.h         # pid and slew gains
│   │   └── hardware.h      # motor/sensor declarations
│   └── src/
│       ├── autons/         # auton route files
│       │   └── forward24.cpp
│       ├── mechs/          # robot mechanisms (lift, intake, etc.)
│       ├── autons.cpp      # selector array & scoring helpers
│       ├── controls.cpp    # driver loop and button binds
│       └── hardware.cpp    # ports and dimensions
├── include/                # engine headers (don't touch)
└── src/                    # engine code (don't touch)
```

---

## Autonomous

Create a `.cpp` file in `custom/src/autons/`, for example `custom/src/autons/twoPin.cpp`:
```cpp
#include "autons.h"
#include "nav.h"
#include "odom.h"

void twoPin() {
    setPose(0.0, 0.0, 0.0);

    moveToPoint(0.0, 24.0);        // drive forward to pin
    grabCup();                     // helper action from autons.cpp
    reverseTo(0.0, 8.0);           // back away with cup
    turnTo(90.0);                  // align to goal
    dropPin();                     // score
}
```

Declare it in `custom/include/autons.h`:
```cpp
void twoPin();
```

And add it to `autons[]` in `custom/src/autons.cpp`:
```cpp
void (*autons[])() = {
    forward24,
    twoPin
};
```
Change `current_auton_selection` to whichever index you want to run. You can also put shared scoring helpers (like `grabCup()` or `dropPin()`) directly in `custom/src/autons.cpp`.

---

## Mechanisms

Put mechanism code in `custom/src/mechs/` (e.g. `intake.cpp`, `lift.cpp`) and declare their motors in `hardware.h`/`hardware.cpp`. Call your mechanism functions in `custom/src/controls.cpp` for driver control, or inside your auton routes.

---

## Updating Rider

To update the engine without touching your `custom/` files:
1. Open the VS Code command palette (`Ctrl+Shift+P` / `Cmd+Shift+P`).
2. Run **Tasks: Run Task** $\rightarrow$ **Update Rider Engine**.

---

## FAQ

**Do I need tracking wheels?**  
No. If you set `using_vert_tracker = false` and `using_horiz_tracker = false`, it falls back to motor encoders and the IMU.

**What units does this use?**  
Inches for distance, degrees for angles (0° is north/forward, clockwise positive), and volts (-12 to 12) for motor output.

**Where do I tune PID?**  
In `custom/include/gains.h`. Drive, turn, and slew values are all there.

---

## License

MIT License. Created by **Vachan Bhogi (2715N)**.
