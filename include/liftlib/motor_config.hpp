#pragma once

#include <cstdint>

#include "api.h"

namespace liftlib {

struct MotorConfig {
	std::int8_t port;
	float gear_ratio = 1;
	pros::motor_brake_mode_e brakeType = pros::E_MOTOR_BRAKE_COAST;
	pros::MotorGears gearset = pros::MotorGears::green;
};

}  // namespace liftlib
