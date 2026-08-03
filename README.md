# liftlib

A PROS library for position-controlled VEX V5 lifts. Drives one or more motor
groups to target positions with PID, and coordinates several of them as a single
multi-stage lift.

## Features

- Motor groups with per-motor gear ratios and brake modes
- PID with integral clamping, integral zone, output limiting, and slew
- Output limits derived automatically from each motor's gearset
- Blocking and async moves, with timeouts on every move
- Soft position limits per subsystem
- Multi-stage coordination through `Lift`

## Installation

Build the template and apply it to a PROS project:

```sh
make template
pros c fetch liftlib@1.0.0.zip
pros c apply liftlib
```

Then include the umbrella header:

```cpp
#include "liftlib/liftlib.hpp"
```

## Usage

### A single subsystem

```cpp
MotorConfig left{1, 1.0f, pros::E_MOTOR_BRAKE_HOLD};
MotorConfig right{-2, 1.0f, pros::E_MOTOR_BRAKE_HOLD};

PID armPid(1.5f, 0.01f, 0.5f, 2.0f);

// Soft limits keep targets between 0 and 90 degrees.
Subsystem arm({left, right}, armPid, 0, 90);

void autonomous() {
    arm.initialize();
    arm.moveTo(45, false, 2000);  // blocking, 2 s timeout
}
```

`gear_ratio` converts motor degrees into whatever unit you want to command in.
Leave it at `1` to work in motor degrees.

### Blocking, async, and timeouts

```cpp
arm.moveTo(45);                  // async, returns immediately
arm.moveTo(45, false);           // blocks until settled
arm.moveTo(45, false, 2000);     // blocks, gives up after 2 s
arm.moveTo(45, true, 2000);      // async, task gives up after 2 s

arm.waitUntilSettled();          // wait for an async move
arm.waitUntilSettled(3000);      // returns false if it times out
arm.stop();                      // cancel and brake
```

Every timeout is in milliseconds. `NO_TIMEOUT` (the default) waits indefinitely.

A move is settled once the error stays inside the PID threshold for
`SETTLE_COUNT` consecutive 20 ms ticks, so passing through the target at speed
does not count as settled.

### A multi-stage lift

```cpp
Subsystem stage1({m1, m2}, stage1Pid, 0, 55);
Subsystem stage2({m3}, stage2Pid);
Subsystem claw({m4}, clawPid);

Lift cascade{&stage1, &stage2, &claw};

void autonomous() {
    cascade.initialize();

    // Every stage, in stage order.
    cascade.moveTo({20, 10, 90}, false, 3000);

    // Or name only the stages you want to move.
    cascade.moveTo({{&stage1, 15}, {&claw, 100}}, false, 3000);
}
```

`Lift` owns the motion while a group move runs: starting one cancels any move
already in progress, including per-stage moves on the stages it is about to
drive. Do not drive a stage directly while a group move that includes it is
running.

### Tuning

`Subsystem` sets the PID output limit from the slowest motor's gearset (100 for
red, 200 for green, 600 for blue), so the controller cannot ask for more velocity
than the motor can deliver. Override it, and the rest of the PID, through
`getPID()`:

```cpp
arm.getPID().setMaxOutput(150);
arm.getPID().setIntegralZone(10);   // only integrate within 10 units of target
arm.getPID().setMaxIntegral(50);
arm.getPID().setSlew(5);            // max output change per tick
```

Start with kP only, raise it until the lift oscillates, then back off and add kD.
Add kI last, with an integral zone, and only to close a steady-state gap.

## Namespacing

Everything lives in the `liftlib` namespace. The umbrella header also exposes
`Lift`, `Subsystem`, `PID`, and `MotorConfig` unqualified for convenience. If
those names collide with another library, opt out:

```cpp
#define LIFTLIB_NO_GLOBAL_NAMES
#include "liftlib/liftlib.hpp"

liftlib::Subsystem arm({left, right}, armPid);
```

## Layout

```
include/liftlib/     public headers (shipped in the template)
  liftlib.hpp        umbrella header
  lift.hpp           multi-stage coordination
  subsystem.hpp      one motor group under PID
  pid.hpp            the controller
  motor_config.hpp   per-motor port, ratio, brake mode, gearset
src/liftlib/         implementation
src/main.cpp         demo entry points, excluded from the template
```

## Requirements

PROS 4 (kernel 4.2.2 or later), C++20 or newer.
