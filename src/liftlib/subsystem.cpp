#include "liftlib/subsystem.hpp"

#include <algorithm>
#include <limits>

Subsystem::Subsystem(std::vector<MotorConfig> motorConfigs, PID pid)
    : motorConfigs(motorConfigs),
      pid(pid),
      minPosition(0),
      maxPosition(0),
      limited(false),
      lastError(std::numeric_limits<float>::max()),
      settled(false) {}

Subsystem::Subsystem(std::vector<MotorConfig> motorConfigs, PID pid, float minPosition,
                     float maxPosition)
    : motorConfigs(motorConfigs),
      pid(pid),
      minPosition(minPosition),
      maxPosition(maxPosition),
      limited(true),
      lastError(std::numeric_limits<float>::max()),
      settled(false) {}

void Subsystem::initialize() {
	for (const MotorConfig& config : motorConfigs) {
		config.motor.tare_position();
		config.motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	}
}

float Subsystem::getPosition() const {
	float currentPosition = 0;
	for (const MotorConfig& config : motorConfigs) {
		float position = config.motor.get_position();
		currentPosition += position * config.gear_ratio;
	}
	return currentPosition / motorConfigs.size();
}

float Subsystem::clampTarget(float target) const {
	return limited ? std::clamp(target, minPosition, maxPosition) : target;
}

void Subsystem::update(float target) {
	lastError = clampTarget(target) - getPosition();
	settled = pid.isSettled(lastError);

	float output = pid.calculate(lastError);
	for (const MotorConfig& config : motorConfigs) {
		config.motor.move_velocity(output);
	}
}

bool Subsystem::isSettled() const {
	return settled;
}

void Subsystem::reset() {
	pid.reset();
	lastError = std::numeric_limits<float>::max();
	settled = false;
}

void Subsystem::brake() {
	for (const MotorConfig& config : motorConfigs) {
		config.motor.set_brake_mode(config.brakeType);
		config.motor.brake();
	}
}

void Subsystem::moveTo(float target) {
	reset();
	while (!isSettled()) {
		update(target);
		pros::delay(20);
	}
	brake();
}
