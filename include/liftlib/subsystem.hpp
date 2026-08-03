#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

#include "api.h"
#include "liftlib/motor_config.hpp"
#include "liftlib/pid.hpp"

namespace liftlib {

struct MotorSlot final {
	pros::Motor motor;
	MotorSlot(std::int8_t port, pros::MotorGears gearset) : motor(port, gearset) {}
};

class Subsystem {
   protected:
	std::vector<MotorConfig> motorConfigs;
	std::vector<MotorSlot> motors;
	PID pid;

	float minPosition;
	float maxPosition;

	bool limited;

   public:
	static constexpr std::uint32_t LOOP_DELAY_MS = 20;
	static constexpr int SETTLE_COUNT = 5;
	static constexpr std::uint32_t NO_TIMEOUT = 0;

	Subsystem(std::vector<MotorConfig> motorConfigs, PID pid);

	Subsystem(std::vector<MotorConfig> motorConfigs, PID pid, float minPosition, float maxPosition);

	Subsystem(const Subsystem&) = delete;
	Subsystem& operator=(const Subsystem&) = delete;

	virtual ~Subsystem();

	virtual void initialize();

	virtual float getPosition() const;

	float clampTarget(float target) const;

	virtual void update(float target);

	virtual bool isSettled() const;

	bool isSettledAt(float target) const;

	virtual void reset();

	virtual void brake();

	/**
	 * Moves the subsystem to a target position.
	 *
	 * When async is true (the default) the motion runs in its own task and this
	 * call returns immediately. When async is false the call blocks until the
	 * subsystem settles or the timeout elapses.
	 *
	 * A timeout of NO_TIMEOUT waits indefinitely.
	 *
	 * Starting a new move cancels any async move already in progress.
	 */
	virtual void moveTo(float target, bool async = true, std::uint32_t timeout = NO_TIMEOUT);

	/**
	 * Blocks until an in-progress async move finishes.
	 *
	 * Returns false when the timeout elapsed first, in which case the move is
	 * cancelled and the motors are braked. A timeout of NO_TIMEOUT waits
	 * indefinitely.
	 */
	bool waitUntilSettled(std::uint32_t timeout = NO_TIMEOUT);

	/** Stops an in-progress async move and brakes the motors. */
	void stop();

	/** True while an async move is still running. */
	bool isMoving() const;

	PID& getPID();

   private:
	float lastError;
	bool settled;
	int settleTicks;

	std::unique_ptr<pros::Task> task;
	std::atomic<bool> cancelRequested;

	void buildMotors();

	void cancelTask();

	void moveToBlocking(float target, std::uint32_t timeout);
};

}  // namespace liftlib
