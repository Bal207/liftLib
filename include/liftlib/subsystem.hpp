#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

#include "api.h"
#include "liftlib/feedforward.hpp"
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
	Feedforward feedforward;

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
	 * Settles the subsystem at its current position: holds against gravity when
	 * feedforward is configured, otherwise brakes.
	 *
	 * The first call latches the position, so calling this every tick keeps the
	 * loop closed around one target. A single call latches the holding output
	 * instead, which is enough to stop a lift sagging after a blocking move.
	 */
	virtual void hold();

	/** Forgets the latched hold position so the next hold() latches afresh. */
	void releaseHold();

	bool isHolding() const;

	/** Drives the motors directly, bypassing the PID. */
	virtual void setOutput(float output);

	bool hasLimits() const;

	float getMinPosition() const;

	float getMaxPosition() const;

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

	/**
	 * Gravity compensation, added to the PID output so the controller does not
	 * have to build up error before it holds position.
	 *
	 * Use Feedforward::constant for a DR4B or cascade, Feedforward::cosine for a
	 * pivoting arm.
	 */
	void setFeedforward(const Feedforward& feedforward);

	Feedforward& getFeedforward();

	/** The holding output at the current position, before any PID term. */
	float holdOutput() const;

   private:
	float lastError;
	bool settled;
	int settleTicks;

	float holdTarget;
	bool holding;

	std::unique_ptr<pros::Task> task;
	std::atomic<bool> cancelRequested;

	void buildMotors();

	void cancelTask();

	void moveToBlocking(float target, std::uint32_t timeout);
};

}  // namespace liftlib
