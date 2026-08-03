#include "liftlib/subsystem.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace liftlib {

namespace {

// Limits the velocity based on what the gearset is
float velocityLimit(pros::MotorGears gearset) {
	switch (gearset) {
		case pros::MotorGears::red:
			return 100;
		case pros::MotorGears::blue:
			return 600;
		default:
			return 200;
	}
}

}  

Subsystem::Subsystem(std::vector<MotorConfig> motorConfigs, PID pid)
    : motorConfigs(motorConfigs),
      pid(pid),
      minPosition(0),
      maxPosition(0),
      limited(false),
      lastError(std::numeric_limits<float>::max()),
      settled(false),
      settleTicks(0),
      cancelRequested(false) {
	buildMotors();
}

Subsystem::Subsystem(std::vector<MotorConfig> motorConfigs, PID pid, float minPosition,
                     float maxPosition)
    : motorConfigs(motorConfigs),
      pid(pid),
      minPosition(std::min(minPosition, maxPosition)),
      maxPosition(std::max(minPosition, maxPosition)),
      limited(true),
      lastError(std::numeric_limits<float>::max()),
      settled(false),
      settleTicks(0),
      cancelRequested(false) {
	buildMotors();
}


void Subsystem::buildMotors() {
	float limit = 0;
	motors.reserve(motorConfigs.size());
	for (const MotorConfig& config : motorConfigs) {
		motors.emplace_back(config.port, config.gearset);
		float motorLimit = velocityLimit(config.gearset);
		limit = limit == 0 ? motorLimit : std::min(limit, motorLimit);
	}

	if (limit > 0 && pid.getMaxOutput() == 0) {
		pid.setMaxOutput(limit);
	}
}

void Subsystem::initialize() {
	for (std::size_t i = 0; i < motors.size(); i++) {
		motors[i].motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
		motors[i].motor.set_brake_mode(motorConfigs[i].brakeType);
		motors[i].motor.tare_position();
	}
}

float Subsystem::getPosition() const {
	float total = 0;
	int counted = 0;
	for (std::size_t i = 0; i < motors.size(); i++) {
		double position = motors[i].motor.get_position();
		if (position == PROS_ERR_F || std::isnan(position)) {
			continue;
		}
		total += static_cast<float>(position) * motorConfigs[i].gear_ratio;
		counted++;
	}
	return counted > 0 ? total / counted : 0;
}

float Subsystem::clampTarget(float target) const {
	return limited ? std::clamp(target, minPosition, maxPosition) : target;
}

void Subsystem::update(float target) {
	lastError = clampTarget(target) - getPosition();

	if (pid.isSettled(lastError)) {
		if (settleTicks < SETTLE_COUNT) {
			settleTicks++;
		}
	} else {
		settleTicks = 0;
	}
	settled = settleTicks >= SETTLE_COUNT;

	float output = pid.calculate(lastError);
	for (const MotorSlot& slot : motors) {
		slot.motor.move_velocity(static_cast<std::int32_t>(output));
	}
}

bool Subsystem::isSettled() const {
	return settled;
}

bool Subsystem::isSettledAt(float target) const {
	return pid.isSettled(clampTarget(target) - getPosition());
}

void Subsystem::reset() {
	pid.reset();
	lastError = std::numeric_limits<float>::max();
	settled = false;
	settleTicks = 0;
}

void Subsystem::brake() {
	for (std::size_t i = 0; i < motors.size(); i++) {
		motors[i].motor.set_brake_mode(motorConfigs[i].brakeType);
		motors[i].motor.brake();
	}
}

void Subsystem::moveToBlocking(float target, std::uint32_t timeout) {
	reset();

	std::uint32_t start = pros::millis();
	std::uint32_t now = start;

	while (!isSettled() && !cancelRequested.load()) {
		if (timeout != NO_TIMEOUT && pros::millis() - start >= timeout) {
			break;
		}
		update(target);
		pros::Task::delay_until(&now, LOOP_DELAY_MS);
	}

	brake();
}

void Subsystem::moveTo(float target, bool async, std::uint32_t timeout) {
	cancelTask();

	if (!async) {
		cancelRequested.store(false);
		moveToBlocking(target, timeout);
		return;
	}

	cancelRequested.store(false);
	task = std::make_unique<pros::Task>(
	    [this, target, timeout]() { moveToBlocking(target, timeout); });
}

void Subsystem::cancelTask() {
	if (task == nullptr) {
		return;
	}

	cancelRequested.store(true);
	task->join();
	task.reset();
	cancelRequested.store(false);
}

bool Subsystem::waitUntilSettled(std::uint32_t timeout) {
	if (task == nullptr) {
		return true;
	}

	if (timeout == NO_TIMEOUT) {
		task->join();
		task.reset();
		return true;
	}
	std::uint32_t start = pros::millis();
	std::uint32_t now = start;
	while (isMoving()) {
		if (pros::millis() - start >= timeout) {
			stop();
			return false;
		}
		pros::Task::delay_until(&now, LOOP_DELAY_MS);
	}

	task->join();
	task.reset();
	return true;
}

void Subsystem::stop() {
	cancelTask();
	brake();
}

bool Subsystem::isMoving() const {
	return task != nullptr && task->get_state() != pros::E_TASK_STATE_DELETED;
}

PID& Subsystem::getPID() {
	return pid;
}

Subsystem::~Subsystem() {
	cancelTask();
}

}  // namespace liftlib
