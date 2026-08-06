#include "liftlib/subsystem.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

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

constexpr float DEFAULT_FEEDFORWARD_HEADROOM = 0.8f;

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
      holdTarget(0),
      holding(false),
      outputMode(OutputMode::Voltage),
      derivedOutputLimit(0),
      torqueSharing(false),
      feedforwardHeadroom(DEFAULT_FEEDFORWARD_HEADROOM),
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
      holdTarget(0),
      holding(false),
      outputMode(OutputMode::Voltage),
      derivedOutputLimit(0),
      torqueSharing(false),
      feedforwardHeadroom(DEFAULT_FEEDFORWARD_HEADROOM),
      cancelRequested(false) {
	buildMotors();
}

float Subsystem::velocityCeiling() const {
	// The slowest cartridge in the group sets the ceiling, since commanding
	// past it would desynchronise a group of mixed gearings.
	float limit = 0;
	for (const MotorConfig& config : motorConfigs) {
		const float motorLimit = velocityLimit(config.gearset);
		limit = limit == 0 ? motorLimit : std::min(limit, motorLimit);
	}
	return limit;
}

float Subsystem::outputLimit() const {
	return outputMode == OutputMode::Voltage ? VOLTAGE_OUTPUT_LIMIT : velocityCeiling();
}

void Subsystem::buildMotors() {
	motors.reserve(motorConfigs.size());
	for (const MotorConfig& config : motorConfigs) {
		motors.emplace_back(config.port, config.gearset);
	}

	buildOutputScales();

	const float limit = outputLimit();
	if (limit > 0 && pid.getMaxOutput() == 0) {
		pid.setMaxOutput(limit);
		derivedOutputLimit = limit;
	}
}

void Subsystem::buildOutputScales() {
	outputScales.assign(motorConfigs.size(), 1.0f);

	// Sharing only means something when the group mixes types. In a group of
	// one type every motor would get the same scale, which is what it already
	// has.
	if (!torqueSharing || !isMixedGroup()) {
		return;
	}

	for (std::size_t i = 0; i < motorConfigs.size(); i++) {
		if (motorConfigs[i].type == MotorType::W5_5) {
			outputScales[i] = W5_5_TORQUE_FRACTION;
		}
	}
}

void Subsystem::setTorqueSharing(bool enabled) {
	torqueSharing = enabled;
	buildOutputScales();
}

bool Subsystem::getTorqueSharing() const {
	return torqueSharing;
}

bool Subsystem::isMixedGroup() const {
	if (motorConfigs.empty()) {
		return false;
	}

	const MotorType first = motorConfigs.front().type;
	for (const MotorConfig& config : motorConfigs) {
		if (config.type != first) {
			return true;
		}
	}
	return false;
}

std::size_t Subsystem::motorCountOfType(MotorType type) const {
	std::size_t count = 0;
	for (const MotorConfig& config : motorConfigs) {
		if (config.type == type) {
			count++;
		}
	}
	return count;
}

void Subsystem::setOutputMode(OutputMode mode) {
	if (mode == outputMode) {
		return;
	}

	// Whether the limit still belongs to us has to be judged against the old
	// mode, before anything changes.
	const bool limitIsOurs = pid.getMaxOutput() == derivedOutputLimit;

	outputMode = mode;

	const float limit = outputLimit();
	if (limit <= 0) {
		// Velocity mode with no motors to read a cartridge from. Clear the
		// derived limit rather than leaving one carried over from the units we
		// just left, which would cap the controller at a meaningless number.
		if (limitIsOurs) {
			pid.setMaxOutput(0);
			derivedOutputLimit = 0;
		}
		return;
	}

	// Only rescale a limit this subsystem chose. A value the user set means
	// something to them that does not survive a change of units, so leave it
	// and let the retune they already have to do cover it.
	if (limitIsOurs) {
		pid.setMaxOutput(limit);
		derivedOutputLimit = limit;
	}
}

OutputMode Subsystem::getOutputMode() const {
	return outputMode;
}

void Subsystem::setPositionSource(PositionSource source) {
	// An async move calls the source every tick from its own task. Replacing a
	// std::function destroys the old target, so swapping it under a running
	// move would be a use-after-free rather than a merely stale read.
	cancelTask();
	positionSource = std::move(source);
}

void Subsystem::clearPositionSource() {
	cancelTask();
	positionSource = nullptr;
}

bool Subsystem::hasPositionSource() const {
	return positionSource != nullptr;
}

void Subsystem::initialize() {
	for (std::size_t i = 0; i < motors.size(); i++) {
		motors[i].motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
		motors[i].motor.set_brake_mode(motorConfigs[i].brakeType);

		// Taring is meaningless against an external sensor: it would move the
		// encoder zero without moving the zero the subsystem actually reads,
		// silently shifting the soft limits.
		if (positionSource == nullptr) {
			motors[i].motor.tare_position();
		}
	}
}

float Subsystem::getPosition() const {
	if (positionSource != nullptr) {
		const float position = positionSource();
		return std::isnan(position) ? 0 : position;
	}

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
	// Read the position once: two reads a tick cost twice the port traffic and
	// can disagree, which would feed the PID and the feedforward slightly
	// different pictures of where the mechanism is.
	updateAt(target, getPosition());
}

void Subsystem::updateAt(float target, float position) {
	lastError = clampTarget(target) - position;

	if (pid.isSettled(lastError)) {
		if (settleTicks < SETTLE_COUNT) {
			settleTicks++;
		}
	} else {
		settleTicks = 0;
	}
	settled = settleTicks >= SETTLE_COUNT;

	const float limit = pid.getMaxOutput();

	// Cap the gravity term below the output limit so the PID always keeps some
	// authority. Without this a heavy mechanism whose kG approaches the limit
	// leaves the controller unable to push any further in that direction.
	float gravity = feedforward.calculate(position);
	if (limit > 0) {
		const float ceiling = limit * feedforwardHeadroom;
		gravity = std::clamp(gravity, -ceiling, ceiling);
	}

	float output = pid.calculate(lastError) + gravity;

	if (limit > 0) {
		output = std::clamp(output, -limit, limit);
	}

	writeOutput(output);
}

void Subsystem::writeOutput(float output) const {
	const float limit = outputLimit();
	if (limit > 0) {
		output = std::clamp(output, -limit, limit);
	}

	// outputScales is all ones unless torque sharing is on in a mixed group, so
	// the usual path is a multiply by 1.0f rather than a branch per motor.
	if (outputMode == OutputMode::Voltage) {
		// -127..127 maps onto the motor's full millivolt range.
		const float millivolts = output * (MAX_MILLIVOLTS / VOLTAGE_OUTPUT_LIMIT);
		for (std::size_t i = 0; i < motors.size(); i++) {
			motors[i].motor.move_voltage(
			    static_cast<std::int32_t>(millivolts * outputScales[i]));
		}
		return;
	}

	for (std::size_t i = 0; i < motors.size(); i++) {
		motors[i].motor.move_velocity(static_cast<std::int32_t>(output * outputScales[i]));
	}
}

void Subsystem::setFeedforwardHeadroom(float fraction) {
	feedforwardHeadroom = std::clamp(fraction, 0.0f, 1.0f);
}

float Subsystem::getFeedforwardHeadroom() const {
	return feedforwardHeadroom;
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
	holding = false;
}

void Subsystem::brake() {
	for (std::size_t i = 0; i < motors.size(); i++) {
		motors[i].motor.set_brake_mode(motorConfigs[i].brakeType);
		motors[i].motor.brake();
	}
}

void Subsystem::hold() {
	if (!feedforward.isEnabled()) {
		brake();
		return;
	}

	// One read serves both the latch and the loop, so a position source is
	// called once per tick and the latched target cannot be offset from the
	// position the controller then measures against it.
	const float position = getPosition();

	// Latch the position on the first call so repeated calls close the loop
	// around one target rather than chasing wherever the mechanism has drifted.
	if (!holding) {
		holdTarget = position;
		holding = true;
	}

	updateAt(holdTarget, position);
}

void Subsystem::releaseHold() {
	holding = false;
}

bool Subsystem::isHolding() const {
	return holding;
}

void Subsystem::setFeedforward(const Feedforward& feedforward) {
	this->feedforward = feedforward;
}

Feedforward& Subsystem::getFeedforward() {
	return feedforward;
}

float Subsystem::holdOutput() const {
	float gravity = feedforward.calculate(getPosition());

	// Report the value update() would actually apply, so a caller that biases
	// around the holding output sees the same number the controller uses.
	const float limit = pid.getMaxOutput();
	if (limit > 0) {
		const float ceiling = limit * feedforwardHeadroom;
		gravity = std::clamp(gravity, -ceiling, ceiling);
	}
	return gravity;
}

void Subsystem::setOutput(float output) {
	float limit = pid.getMaxOutput();
	if (limit > 0) {
		output = std::clamp(output, -limit, limit);
	}

	writeOutput(output);
}

bool Subsystem::hasLimits() const {
	return limited;
}

float Subsystem::getMinPosition() const {
	return minPosition;
}

float Subsystem::getMaxPosition() const {
	return maxPosition;
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

	if (cancelRequested.load()) {
		brake();
	} else {
		hold();
	}
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
