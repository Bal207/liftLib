# liftLib

An extremely easy to use yet dynamic library for vexV5 Lifts.
- Easy to tune, setup and use for auton routines
- Great for organizing your different subsystems for custom control

## Features

- Motor groups with per-motor gear ratios and brake modes
- Voltage control by default, on the familiar -127 to 127 scale
- PID with integral clamping, integral zone, output limiting, and slew
- Position from the motor encoders or any sensor you plug in
- Blocking and async moves, with timeouts on every move
- Soft position limits per subsystem
- Multi-stage coordination through `Lift`, pistons included
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

### 5.5W motors

State the type when a port holds a 5.5W (EXP) motor rather than the 11W:

```cpp
MotorConfig small{7, 1.0f, pros::E_MOTOR_BRAKE_HOLD,
                  pros::MotorGears::green, liftlib::MotorType::W5_5};
```

The two motors differ in torque, not speed. A 5.5W with a 200 RPM cartridge free
spins as fast as an 11W with a green one, so the velocity ceiling is unchanged;
what differs is that it stalls at roughly half the torque and heats up sooner.

Because of that, **a 5.5W needs no special handling on its own**. In voltage mode
a full command is ±12000 mV to either motor, and each contributes whatever it
physically can. A subsystem of nothing but 5.5W motors is commanded exactly like
one of 11W motors.

Mixing the two in one group also works by default: both get the same command, so
the 11W naturally does more of the work. Turn on torque sharing only if the 5.5W
is the one overheating:

```cpp
lift.setTorqueSharing(true);   // scales the 5.5W's command to half
```

That makes the 11W carry proportionally more, at the cost of some total output.
It does nothing in a group of one motor type, so it is safe to set
unconditionally.

`isMixedGroup()` and `motorCountOfType()` report what the group holds.

`MotorType` is only reachable as `liftlib::MotorType`, since `pros` has a type of
the same name.

### Reading position from a sensor

By default a subsystem averages its motor encoders. A rotation sensor or
potentiometer mounted on the joint sees the mechanism directly, so it does not
accumulate the backlash and slip between the motor and the load:

```cpp
pros::Rotation armSensor(11);

arm.setPositionSource([&] { return armSensor.get_angle() / 100.0f; });
```

The callback returns the position in the same units you use for targets and soft
limits. Gear ratios are not applied to it, since the sensor already reads the
output side of the gearing.

Set the source before `initialize()`. With a source configured, `initialize()`
stops taring the motors, because taring would move the encoder zero without
moving the zero the subsystem actually reads, silently shifting the soft limits.

The callback runs once per control tick from whichever task is driving the
subsystem, so keep it cheap, and make sure anything it captures by reference
outlives the subsystem. `clearPositionSource()` goes back to the encoders.

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

### Output units

Output is a -127 to 127 command applied as voltage, the same scale
`pros::Motor::move` uses. 127 is full power in the direction you asked for.

Voltage is the default because it is open loop: the only controller in the path
is the one in this library. In velocity mode the V5 firmware runs its own loop
inside yours, which fights a tightly tuned PID and is a common cause of a lift
that feels sluggish no matter how it is tuned.

Switch a subsystem over if you want the motor holding a velocity for you:

```cpp
arm.setOutputMode(OutputMode::Velocity);   // output is now RPM
```

In velocity mode the range is the cartridge's: 100 for red, 200 for green, 600
for blue, taken from the slowest motor in the group.

The two modes have different units, so **gains do not carry across a mode
change**. Retune kP, kD, and kG after switching. `setOutputMode` rescales the
PID's output limit for you when it still holds the value the subsystem derived,
but a limit you set yourself is left alone, since only you know what it meant.

### Driving a subsystem by hand

`setOutput` skips the PID and commands the motors directly, which is what you
want for driver control. The value is clamped to the same output limit the PID
uses, so you cannot ask for more than the mode allows:

```cpp
void opcontrol() {
    while (true) {
        int up = master.get_digital(DIGITAL_L1);
        int down = master.get_digital(DIGITAL_L2);

        if (up) {
            arm.setOutput(127);
        } else if (down) {
            arm.setOutput(-127);
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
    arm.setOutput(127);
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

### Pneumatics

A `Piston` is a `Lift` stage like any other, so a clamp or a set of wings can sit
alongside the motorised stages and be named in a conditional action:

```cpp
Piston clamp(PistonConfig{'A'});
Piston wings({{'B'}, {'C', true}});   // two solenoids, the second reversed

Lift robot{&arm, &claw, &clamp, &wings};

clamp.extend();
clamp.retract();
clamp.toggle();
clamp.isExtended();
```

Set `reversed` when a cylinder retracts on a high signal, and `startExtended`
for the state the port should be driven to at startup.

A piston has no position, so it never takes a numeric target. Target lists cover
only the stages that have one, in the order you gave them:

```cpp
Lift robot{&arm, &clamp, &claw};

robot.moveTo({20, 90});   // arm and claw; the clamp is skipped, not counted
```

`stageCount()` counts everything, `positionalStageCount()` counts only what can
take a target. `getStage()` indexes the first, `getPositionalStage()` the second.

There is no sensor on a cylinder, so a piston reports settled once it has had
time to travel. That time is the one number worth setting per mechanism:

```cpp
Piston clamp(PistonConfig{'A'}, 300);   // 300 ms of travel
clamp.setActuationTime(180);

clamp.extend(false);            // blocks for the actuation time
clamp.waitUntilSettled(500);    // or wait on an async one
```

`brake()` and `hold()` do nothing on a piston: air holds the cylinder without
power, and dropping the signal would move it rather than stop it. `stop()` does
not cut the signal either, it just reports the travel as over so a stopped lift
does not sit waiting on a stage that cannot be stopped.

### Conditional actions

A conditional action runs a piece of code when a condition becomes true. Use it
for the things you would otherwise scatter through opcontrol: stop the lift at a
hard stop, retract the claw when a sensor trips, park the arm when a button is
held.

```cpp
lift.addConditionalAction(
    [] { return limitSwitch.get_value(); },   // condition
    [&] { lift.stop(); },                     // what to do
    Precedence::Absolute);                    // how hard it pushes

lift.watchConditions();   // start polling in its own task
```

Conditions are checked every 20 ms. An action fires on the change from false to
true, not for as long as the condition stays true, so a switch that stays pressed
fires once. Let it go false and it arms again.

`addConditionalAction` gives back an id you can use later:

```cpp
int id = lift.addConditionalAction(cond, act, Precedence::Medium);

lift.removeConditionalAction(id);
lift.clearConditionalActions();
lift.conditionalActionCount();
```

If you would rather not spend a task on polling, call `checkConditions()` from a
loop you already have:

```cpp
while (true) {
    lift.checkConditions();
    pros::delay(20);
}
```

#### Which stages an action touches

Precedence is judged per subsystem, not across the whole lift. Name the stages an
action drives and it is only ever held up by moves that touch those same stages:

```cpp
lift.addConditionalAction(cond, act, Precedence::Medium, {&claw});
```

A claw action carries on regardless of what the lift stage is doing, even during
a blocking move, because they do not share a subsystem. Only an overlap matters,
so an action on `{&claw, &liftStage}` does wait for a move on the lift stage.

Leave the stage list off and the action counts as touching everything, which is
what you want for a full stop:

```cpp
lift.addConditionalAction(emergency, [&]{ lift.stop(); }, Precedence::Absolute);
```

Stages that are not part of the lift are rejected, and `addConditionalAction`
returns 0.

#### Precedence

Precedence decides what happens when a condition fires while the stages it wants
are already busy.

| Level | Stages free | Busy with an async move | Busy with a blocking move |
| --- | --- | --- | --- |
| `Low` | runs | dropped | dropped |
| `Medium` | runs | runs after the move | runs after the move |
| `High` | runs | runs straight away | runs after the move |
| `Absolute` | runs | runs straight away | runs straight away |

`Low` is the default and the most polite. It only runs when its stages are free,
and if they are busy when the condition fires the action is dropped rather than
saved for later. Good for things that stop being worth doing the moment they are
late, like a convenience move that the driver has already overridden.

`Medium` waits its turn. The action is remembered and runs once the move on those
stages finishes, so nothing gets lost.

`High` cuts in front of an async move but still respects a blocking one, which
matters because a blocking move is usually an auton routine part way through a
sequence.

`Absolute` runs no matter what, and is the one level that ignores stage scoping
entirely. Save it for safety: a hard stop, a limit switch, anything where waiting
would break something.

`High` is allowed to run during an async move but does not cancel it for you. If
the action needs the mechanism to itself, call `lift.stop()` inside it.

Actions run on the polling task, so keep them short. Calling `moveTo`, `stop`, or
adding and removing other actions from inside an action is fine.

### Tuning

`Subsystem` sets the PID output limit from the output mode: 127 in voltage mode,
or the slowest motor's gearset in velocity mode. Override it, and the rest of the
PID, through `getPID()`:

```cpp
arm.getPID().setMaxOutput(100);
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

The gravity term and the PID share one output range, so a large `kG` on a heavy
mechanism can eat most of it and leave the controller unable to push any further
in that direction. The feedforward is capped at 80% of the output limit to stop
that, which always leaves the PID a fifth of the range:

```cpp
arm.setFeedforwardHeadroom(0.9f);   // let gravity claim more
```

Needing much above the default usually means the mechanism is geared too fast for
what it carries. `holdOutput()` reports the capped value, so it always matches
what the controller is actually applying.

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
for a gentler approach, which usually works best for most lifts.

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
  mechanism.hpp      what a Lift stage has to be
  subsystem.hpp      one motor group under PID
  piston.hpp         a pneumatic stage
  pid.hpp            the controller
  feedforward.hpp    gravity compensation
  autotuner.hpp      relay-feedback PID tuning
  motor_config.hpp   per-motor port, ratio, brake mode, gearset
  output_mode.hpp    voltage or velocity output
src/liftlib/         implementation
src/main.cpp         demo entry points, excluded from the template
```

## Upgrading

Output moved from velocity to voltage, so **existing gains do not carry over**.
A kP that was tuned against a 0-200 velocity range now drives a 0-127 voltage
range, and the motor no longer runs its own loop underneath yours.

Retune, or put the subsystem back the way it was:

```cpp
arm.setOutputMode(OutputMode::Velocity);
```

`kG` moves with it, so retune gravity compensation too, and autotune afterwards
rather than reusing a stored result.

`Lift` now holds `Mechanism*` rather than `Subsystem*`, so that pistons can be
stages. Existing code that passes subsystems is unaffected, but the accessors
split in two: `getStage()` returns a `Mechanism*` covering every stage, and
`getPositionalStage()` returns a `Subsystem*` covering the ones that take
targets. Code that used the `Subsystem*` from `getStage()` should move to
`getPositionalStage()`.

## Requirements

PROS 4 (4.2.2 onward) and C++20 or newer.
