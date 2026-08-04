# liftLib

An extremely easy to use yet dynamic library for vexV5 Lifts.
- Easy to tune, setup and use for auton routines
- Great for organizing your different subsystems for custom control

## Features

- Motor groups with per-motor gear ratios and brake modes
- PID with integral clamping, integral zone, output limiting, and slew
- Output limits derived automatically from each motor's gearset
- Blocking and async moves, with timeouts on every move
- Soft position limits per subsystem
- Multi-stage coordination through `Lift`
- Gravity compensation for lifts and pivoting arms
- Automatic PID tuning by relay feedback

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
The gear ratio can be dynamically used depending on the use case (winches, gears, etc.) It is simply a scaling factor
multiplied by the motor encoder units (degrees). (motor position * gear ratio).

```cpp
MotorConfig left{1, 1.0f, pros::E_MOTOR_BRAKE_HOLD}; // port, gear ratio, brake mode
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

### Driving a subsystem by hand

`setOutput` skips the PID and commands the motors directly, which is what you
want for driver control. The value is clamped to the same output limit the PID
uses, so you cannot ask for more than the gearset can give:

```cpp
void opcontrol() {
    while (true) {
        int up = master.get_digital(DIGITAL_L1);
        int down = master.get_digital(DIGITAL_L2);

        if (up) {
            arm.setOutput(150);
        } else if (down) {
            arm.setOutput(-150);
        } else {
            arm.hold();       // stays put instead of dropping
        }

        pros::delay(20);
    }
}
```

`setOutput` does not respect the soft limits, since it does not know where you
are heading. If you want driver control to stop at the ends, check the position
yourself:

```cpp
if (up && arm.getPosition() < arm.getMaxPosition()) {
    arm.setOutput(150);
}
```

`hasLimits()`, `getMinPosition()`, and `getMaxPosition()` report what the
subsystem was built with.

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
already in progress, including seperate stage movements on the stages it is about to
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
If necessary you can add KI last for steady state error, and even use an integral zone for further benefits.

Gains can be changed while the robot is running, which is useful if you want a
screen button or a controller shortcut to try a value without rebuilding:

```cpp
arm.getPID().setGains(2.0f, 0.01f, 8.0f);
arm.getPID().setThreshold(1.5f);   // how close counts as settled

float kp = arm.getPID().getKP();   // also getKI, getKD, getThreshold
```

`setGains` clears the integral and the previous error, so a change mid move does
not carry stale state into the new gains.

### Gravity compensation

PID alone can only fight gravity by first building up error, so a lift sags below
its target and drifts once the move ends. A feedforward term adds the output
needed to hold the load, so the PID only has to correct what is left.

For a DR4B, cascade, or anything travelling straight up, the load barely changes
with position, so a constant works:

```cpp
arm.setFeedforward(Feedforward::constant(12));   // output units to hold station
```

For a pivoting arm the load peaks when the arm is horizontal, so scale by the
cosine of the angle:

```cpp
// 18 units to hold level, level at position 0, position already in degrees
arm.setFeedforward(Feedforward::cosine(18, 0, 1));
```

The third argument converts your position units into degrees, so pass `1` when
the subsystem already works in arm degrees.

To find `kG`, raise the mechanism to mid-range and increase the value until it
just stops sinking. Overshoot it and the lift creeps upward.

With feedforward set, a finished move ends in `hold()` rather than a plain brake.
The loop stays closed around the position it stopped at, so an imperfect `kG` is
corrected instead of drifting. An explicitly stopped or cancelled move still
brakes, because that means "stop", not "hold here".

You can also hold the mechanism yourself. The first call to `hold()` remembers
where the subsystem is, and every call after that drives it back to that spot, so
calling it in a loop is what actually pins a lift in place:

```cpp
while (holdingTheLift) {
    arm.hold();
    pros::delay(20);
}
arm.releaseHold();   // forget the latched spot, so the next hold() picks a new one
arm.brake();
```

`isHolding()` tells you whether a spot is currently latched. `moveTo` and
`reset()` clear it for you, so a new move always starts fresh.

Without feedforward configured, `hold()` just brakes, which means you can call it
unconditionally and it does the right thing either way.

`holdOutput()` returns the output the feedforward is asking for at the current
position. It is handy when you want to check your `kG` on the screen:

```cpp
pros::lcd::print(1, "hold %.1f", arm.holdOutput());
```

For a `Lift`, `hold()` holds every stage at once, each with its own feedforward:

```cpp
cascade.hold();
```

Gravity compensation is per subsystem, so a cascade that carries load on the
bottom stage and nothing on the claw only needs it on the stage that sags.

### Autotuning

`Autotuner` finds gains for you. It drives the subsystem with a bang-bang output
around a setpoint, measures the oscillation that comes back, and converts it into
PID gains. A run typically converges in five cycles.

The subsystem oscillates for the whole run, so keep clear of it and tune outside
of a match.

```cpp
Subsystem arm({left, right}, armPid, 0, 90);

void tuneArm() {
    arm.initialize();

    Autotuner tuner(arm);
    AutotuneResult result = tuner.run(AutotuneConfig{});

    if (result.success) {
        printf("kP %.3f  kI %.4f  kD %.2f\n", result.kp, result.ki, result.kd);
    } else {
        printf("tuning failed: %s\n", result.error);
    }
}
```

Copy the printed gains into your `PID` so they survive a restart, or apply them
for the current run with `runAndApply`:

```cpp
Autotuner tuner(arm);
tuner.runAndApply(AutotuneConfig{});   // writes the gains into arm's PID
```

On a subsystem with soft limits the default setpoint is the midpoint and the
oscillation is kept inside the limits automatically. Without limits, give it a
setpoint and a travel budget:

```cpp
AutotuneConfig config;
config.useSetpoint = true;
config.setpoint = 30;
config.maxExcursion = 15;   // stop if it strays 15 units from the setpoint
```

Drive strength and the noise band are worked out for you: the relay starts from
the motor gearset limit and grows or shrinks during the run until the swing is
big enough to measure but still well inside the travel budget, and the noise band
comes from the PID settle threshold. A run reports what it settled on in
`result.relayAmplitude`.

The remaining knobs are rarely needed:

```cpp
config.tolerance = 0.1f;      // cycles must agree within 10% to accept the run
config.timeout = 15000;
config.rule = TuningRule::NoOvershoot;
```

Pick the rule to suit the mechanism. `ZieglerNicholsPID` (the default) is
responsive but overshoots a little; `NoOvershoot` and `SomeOvershoot` trade speed
for a gentler approach, which is usually what you want on a lift carrying game
elements.

When a run fails, `result.status` says why:

| Status | Meaning |
| --- | --- |
| `Success` | Gains are in `result` |
| `InvalidConfig` | A config field is out of range; see `result.error` |
| `OutOfRoom` | Limits too tight to oscillate between |
| `NoOscillation` | Never oscillated; the mechanism may be stuck |
| `ExcursionExceeded` | Travelled too far; give it more room or set `maxExcursion` |
| `TimedOut` | Did not converge before `timeout` |
| `Cancelled` | `cancel()` was called |

`result` also carries `ultimateGain`, `ultimatePeriod`, `cyclesMeasured`, and
`elapsed` if you want to see what the run actually measured.

`run` blocks until it finishes, so if you want a way out mid run, put it in a task
and call `cancel()` from the controller loop:

```cpp
Autotuner tuner(arm);

pros::Task tuning([&] { tuner.runAndApply(AutotuneConfig{}); });

while (tuner.isRunning()) {
    if (master.get_digital(DIGITAL_B)) {
        tuner.cancel();
    }
    pros::delay(20);
}
```

A cancelled run brakes the motors and comes back with `AutotuneStatus::Cancelled`.

Tune `kG` before you autotune, not after. The tuner swings the relay around the
holding output, so with the right `kG` a loaded lift rises and falls evenly
instead of sagging through the whole run.

Relay tuning gets you a good starting point rather than a perfect answer. The
underlying method approximates the mechanism's response, so treat the gains as a
solid base and hand trim from there if you need the last few percent.

## Namespacing

Everything lives in the `liftlib` namespace. The umbrella header also exposes
`Lift`, `Subsystem`, `PID`, `MotorConfig`, and the `Autotune*` types unqualified
for convenience. If those names collide with another library, opt out:

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
  feedforward.hpp    gravity compensation
  autotuner.hpp      relay-feedback PID tuning
  motor_config.hpp   per-motor port, ratio, brake mode, gearset
src/liftlib/         implementation
src/main.cpp         demo entry points, excluded from the template
```

## Requirements

PROS 4 (4.2.2 onward) and C++20 or newer.
