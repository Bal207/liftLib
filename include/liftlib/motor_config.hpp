#pragma once

#include "api.h"

struct MotorConfig {
	pros::Motor motor;
	float gear_ratio;
	pros::motor_brake_mode_e brakeType = pros::E_MOTOR_BRAKE_COAST;
};
