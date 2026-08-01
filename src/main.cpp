#include "main.h"
#include <vector>
#include <limits>
#include <cmath>

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





class PID {
	private:
		float KP = 0, KI = 0, KD = 0;
		float previousError = 0;
		float integral = 0;
		float threshold = 0;

		
	public:
		PID(float kp, float ki, float kd, float threshold) : KP(kp), KI(ki), KD(kd), threshold(threshold) {}

		float calculate(float setpoint, float actual) {
			return calculate(setpoint - actual);
		}

		float calculate(float error)
		{
			float derivative = error - previousError;
			integral = integral + error;
			float output = KP * error + KI * integral + KD * derivative;
			previousError = error;
			return output;
		}

		void reset() {
			integral = 0;
			previousError = 0;
		}

		bool isSettled(float error) {
			return std::abs(error) < threshold;
		}
		

};


struct MotorConfig
{
	pros::Motor motor;
	float gear_ratio; //Driven/Driver
	pros::motor_brake_mode_e brakeType = pros::E_MOTOR_BRAKE_COAST;
};

pros::Motor claw_motor(-10);
pros::Motor liftMotor1(6, pros::MotorGear::green);
pros::Motor liftMotor2(-7, pros::MotorGear::green);
pros::Motor clawRotation(9);
MotorConfig motor1{claw_motor, 1, pros::E_MOTOR_BRAKE_HOLD};
MotorConfig liftMotorOne{liftMotor1, (0.78 * std::numbers::pi)/360};
MotorConfig liftMotorTwo{liftMotor2, (0.78 * std::numbers::pi)/360};
MotorConfig clawRotationConfig{clawRotation, 5};


PID claw_pid(1, 0, 0, 1);
PID lift_pid(5, 0, 0, 1);
PID claw_rotation_pid(1, 0, 0, 1);


void moveTo(std::vector<MotorConfig>& motorConfigs, PID& pid, float target)
{
	float output;
	float error = std::numeric_limits<float>::max();
	float currentPosition;
	pid.reset();

	while(!pid.isSettled(error)) {
		currentPosition = 0;
		for(const MotorConfig& config : motorConfigs)
		{
			float position = config.motor.get_position();
			currentPosition += position * config.gear_ratio;
		}
		currentPosition /= motorConfigs.size();
		error = target - currentPosition;
		output = pid.calculate(error);

		for(const MotorConfig& config : motorConfigs)
		{
			config.motor.move_velocity(output);
		}
		std::cout << "Error: " << error << " currentPosition: " << currentPosition << " Output " << output << " Actual Motor Input " << liftMotor1.get_current_draw() <<std::endl;

		pros::delay(100);
	}
	for(const MotorConfig& config : motorConfigs)
	{
		config.motor.set_brake_mode(config.brakeType);
		config.motor.brake();
	}
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
void autonomous() {
	std::vector<MotorConfig> clawMotors{motor1};
	std::vector<MotorConfig> liftMotors{liftMotorOne, liftMotorTwo};
	std::vector<MotorConfig> clawRotationMotors{clawRotationConfig};
	claw_motor.tare_position();
	claw_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	liftMotor1.tare_position();
	liftMotor2.tare_position();

	liftMotor1.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	liftMotor2.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

	clawRotation.tare_position();
	clawRotation.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	
	//moveTo(liftMotors, claw_pid, 100);
	//moveTo(liftMotors, lift_pid, 15);
	moveTo(clawRotationMotors, claw_rotation_pid, -90);
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
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}