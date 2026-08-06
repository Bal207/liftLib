#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "api.h"
#include "liftlib/feedforward.hpp"
#include "liftlib/mechanism.hpp"
#include "liftlib/motor_config.hpp"
#include "liftlib/output_mode.hpp"
#include "liftlib/pid.hpp"

namespace liftlib {

struct MotorSlot final {
	pros::Motor motor;
	MotorSlot(std::int8_t port, pros::MotorGears gearset) : motor(port, gearset) {}
};

class Subsystem : public Mechanism {
   protected:
	std::vector<MotorConfig> motorConfigs;
	std::vector<MotorSlot> motors;
	PID pid;
	Feedforward feedforward;

	float minPosition;
	float maxPosition;

	bool limited;

   public:
	using Mechanism::LOOP_DELAY_MS;
	using Mechanism::NO_TIMEOUT;

	static constexpr int SETTLE_COUNT = 5;

	/**
	 * Reads the mechanism's position, in whatever units the subsystem commands.
	 *
	 * Return the position directly; the gear ratios in MotorConfig are not
	 * applied to it, since an external sensor does not sit behind the gearing
	 * the motors do.
	 */
	using PositionSource = std::function<float()>;

	Subsystem(std::vector<MotorConfig> motorConfigs, PID pid);

	Subsystem(std::vector<MotorConfig> motorConfigs, PID pid, float minPosition, float maxPosition);

	~Subsystem() override;

	Subsystem* asSubsystem() override {
		return this;
	}

	void initialize() override;

	virtual float getPosition() const;

	/**
	 * Reads position from a sensor instead of the motor encoders.
	 *
	 * A rotation sensor or potentiometer on the joint sees the mechanism
	 * directly, so it is free of the backlash and slip between the motor and
	 * the load that motor encoders accumulate:
	 *
	 *   arm.setPositionSource([&] { return rotation.get_angle() / 100.0f; });
	 *
	 * The callback runs once per control tick, from whichever task is driving
	 * the subsystem, so it must be cheap and safe to call from a task. Anything
	 * it captures by reference has to outlive the subsystem.
	 *
	 * Units must match the ones used for targets and soft limits. Gear ratios
	 * are not applied, since the sensor is already reading the output side.
	 *
	 * initialize() no longer tares the motors once a source is set, because the
	 * source defines zero and taring would not move it.
	 *
	 * Cancels an async move in progress, since that move would otherwise be
	 * calling the old source from its own task while it is being replaced.
	 * Set the source during setup, not mid-routine.
	 */
	void setPositionSource(PositionSource source);

	/** Reverts to averaging the motor encoders. Cancels an async move. */
	void clearPositionSource();

	bool hasPositionSource() const;

	float clampTarget(float target) const;

	virtual void update(float target);

	bool isSettled() const override;

	bool isSettledAt(float target) const;

	void reset() override;

	void brake() override;

	/**
	 * Settles the subsystem at its current position: holds against gravity when
	 * feedforward is configured, otherwise brakes.
	 *
	 * The first call latches the position, so calling this every tick keeps the
	 * loop closed around one target. A single call latches the holding output
	 * instead, which is enough to stop a lift sagging after a blocking move.
	 */
	void hold() override;

	/** Forgets the latched hold position so the next hold() latches afresh. */
	void releaseHold();

	bool isHolding() const;

	/** Drives the motors directly, bypassing the PID. */
	virtual void setOutput(float output);

	/**
	 * Chooses how output values reach the motors.
	 *
	 * Voltage (the default) treats output as a -127..127 command applied open
	 * loop. Velocity treats it as RPM and hands it to the motor's own
	 * controller.
	 *
	 * Changing the mode rescales what every gain means, so retune kP, kD, and
	 * kG after switching. The PID's output limit is rescaled for you when it
	 * still holds the value this subsystem derived; an explicit setMaxOutput()
	 * is left alone, since only you know what it meant.
	 */
	void setOutputMode(OutputMode mode);

	OutputMode getOutputMode() const;

	/** The largest output the current mode accepts: 127, or the gearset's RPM. */
	float outputLimit() const;

	/**
	 * Splits output between motors by their stall torque.
	 *
	 * Off by default, and only meaningful in a group that mixes an 11W with a
	 * 5.5W. With it off both motors get the same command, so each contributes
	 * whatever it physically can; that is usually what you want, since a 5.5W
	 * commanded to full is already giving everything it has.
	 *
	 * Turn it on when the 5.5W is the one overheating. It scales the smaller
	 * motor's command down so the 11W carries proportionally more of the load,
	 * at the cost of some total output.
	 *
	 * Has no effect on a group of one motor type.
	 */
	void setTorqueSharing(bool enabled);

	bool getTorqueSharing() const;

	/** True when the group holds more than one motor type. */
	bool isMixedGroup() const;

	/** How many motors of a type the group holds. */
	std::size_t motorCountOfType(MotorType type) const;

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
	 * Fraction of the output range the feedforward may claim, 0 to 1.
	 *
	 * The gravity term and the PID term share one output range, so on a heavy
	 * mechanism a large kG can consume most of it and leave the PID unable to
	 * push further in that direction. Capping the feedforward reserves the rest
	 * for the controller.
	 *
	 * Defaults to 0.8, so the PID always keeps at least a fifth of the range.
	 * Raise it towards 1 if the mechanism genuinely needs almost all of the
	 * output just to hold station, though that is usually a sign of being
	 * geared too fast for the load.
	 */
	void setFeedforwardHeadroom(float fraction);

	float getFeedforwardHeadroom() const;

	/**
	 * Blocks until an in-progress async move finishes.
	 *
	 * Returns false when the timeout elapsed first, in which case the move is
	 * cancelled and the motors are braked. A timeout of NO_TIMEOUT waits
	 * indefinitely.
	 */
	bool waitUntilSettled(std::uint32_t timeout = NO_TIMEOUT);

	/** Stops an in-progress async move and brakes the motors. */
	void stop() override;

	/** True while an async move is still running. */
	bool isMoving() const override;

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

	OutputMode outputMode;

	/** Output limit this subsystem derived, so setOutputMode knows to rescale. */
	float derivedOutputLimit;

	bool torqueSharing;

	/**
	 * Per-motor output multiplier, parallel to motorConfigs.
	 *
	 * All ones unless torque sharing is on in a mixed group, so the common case
	 * costs one multiply by 1.0f rather than a branch per motor.
	 */
	std::vector<float> outputScales;

	void buildOutputScales();

	PositionSource positionSource;

	float feedforwardHeadroom;

	std::unique_ptr<pros::Task> task;
	std::atomic<bool> cancelRequested;

	void buildMotors();

	void cancelTask();

	void moveToBlocking(float target, std::uint32_t timeout);

	/**
	 * The body of update(), given a position already read.
	 *
	 * Keeping the read in the caller lets hold() latch and drive from one
	 * sample, so a position source is invoked once per tick rather than twice
	 * with a chance of disagreeing in between.
	 */
	void updateAt(float target, float position);

	/** Sends an output value to the motors in whichever mode is configured. */
	void writeOutput(float output) const;

	/** The gearset velocity ceiling, used as the limit in Velocity mode. */
	float velocityCeiling() const;
};

}  // namespace liftlib
