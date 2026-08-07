#include "main.h"

#include <numbers>
#include <vector>

#include "liftlib/liftlib.hpp"
#include "liftlib/subsystem.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Omkar cant touch anyone");

  pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

constexpr float LIFT_RATIO = (2 * 0.78 * std::numbers::pi) / 360;

MotorConfig motor1{-10, 1, pros::E_MOTOR_BRAKE_HOLD};
MotorConfig liftMotorOne{6, LIFT_RATIO, pros::E_MOTOR_BRAKE_HOLD};
MotorConfig liftMotorTwo{-7, LIFT_RATIO, pros::E_MOTOR_BRAKE_HOLD};
MotorConfig clawRotationConfig{9, 0.2};

PID claw_pid(1, 0, 0, 0.5);
PID lift_pid(2, 0, 0, 0.5);
PID claw_rotation_pid(1, 0, 0, 0.5);

Subsystem liftStage({liftMotorOne, liftMotorTwo}, lift_pid, 0, 55);
Subsystem claw({motor1}, claw_pid, 0, 180);
Subsystem clawRotator({clawRotationConfig}, claw_rotation_pid, -85, 85);

// A clamp on ADI port A. It has no position, so it never takes a target.
Piston clamp(PistonConfig{'A'});

// The lift stage reads its angle from a rotation sensor on the pivot rather
// than the motor encoders, so gear backlash does not show up as position error.
pros::Rotation liftSensor(11);

Lift cascade{&liftStage, &claw, &clawRotator, &clamp};

void tuneLiftStage() {
  liftStage.initialize();

  // Increase until the lift doesnt sag at mid height
  liftStage.setFeedforward(Feedforward::constant(0));

  Autotuner tuner(liftStage);
  AutotuneResult result = tuner.run(AutotuneConfig{});

  if (!result.success) {
    pros::lcd::print(3, "tune failed: %s", result.error);
    return;
  }

  pros::lcd::print(3, "kP %.3f kI %.4f", result.kp, result.ki);
  pros::lcd::print(4, "kD %.2f", result.kd);
  pros::lcd::print(5, "Ku %.2f Tu %.2fs", result.ultimateGain,
                   result.ultimatePeriod);
  pros::lcd::print(6, "%d cycles in %u ms", result.cyclesMeasured,
                   result.elapsed);
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

bool clawIsOverExtended() { return claw.getPosition() > 175; }

void retractClaw() { claw.moveTo(150, false, 500); }

void autonomous() {
  // Read the lift angle from the sensor on the joint. Set this before
  // initialize(), so it knows not to tare the motors against a zero the sensor
  // does not share.
  liftStage.setPositionSource(
      [] { return static_cast<float>(liftSensor.get_angle()) / 100.0f; });

  cascade.initialize();

  // Scoped to the claw, so it still fires while the lift stage is moving.
  cascade.addConditionalAction(clawIsOverExtended, retractClaw, Precedence::Absolute,
                               {&claw});

  // Scoped to the clamp, so it fires even mid-move on the lift.
  cascade.addConditionalAction([] { return liftStage.getPosition() > 40; },
                               [] { clamp.extend(); }, Precedence::High,
                               {&clamp});

  cascade.watchConditions();

  cascade.moveTo({{&liftStage, 15}, {&claw, 180}, {&clawRotator, -90}}, false,
                 3000);

  liftStage.moveTo("Stage 1");

  clamp.retract(false); // blocks for the actuation time

  cascade.stopWatching();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  pros::MotorGroup left_mg({1, -2, 3});   // Creates a motor group with forwards
                                          // ports 1 & 3 and reversed port 2
  pros::MotorGroup right_mg({-4, 5, -6}); // Creates a motor group with forwards
                                          // port 5 and reversed ports 4 & 6

  while (true) {
    pros::lcd::print(0, "%d %d %d",
                     (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                     (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                     (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >>
                         0); // Prints status of the emulated screen LCDs

    // Arcade control scheme
    int dir = master.get_analog(
        ANALOG_LEFT_Y); // Gets amount forward/backward from left joystick
    int turn = master.get_analog(
        ANALOG_RIGHT_X);       // Gets the turn left/right from right joystick
    left_mg.move(dir - turn);  // Sets left motor voltage
    right_mg.move(dir + turn); // Sets right motor voltage
    pros::delay(20);           // Run for 20 ms then update
  }
}
