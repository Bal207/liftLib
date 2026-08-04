#pragma once

#include <atomic>
#include <cstdint>

#include "liftlib/pid.hpp"
#include "liftlib/subsystem.hpp"

namespace liftlib {

/** How the measured oscillation is turned into gains. */
enum class TuningRule {
	ZieglerNicholsP,
	ZieglerNicholsPI,
	ZieglerNicholsPD,
	ZieglerNicholsPID,
	PessenIntegral,
	SomeOvershoot,
	NoOvershoot,
};

/** Why a tuning run stopped. */
enum class AutotuneStatus {
	Success,
	InvalidConfig,
	OutOfRoom,
	NoOscillation,
	ExcursionExceeded,
	TimedOut,
	Cancelled,
};

struct AutotuneConfig {
	/** Position the subsystem oscillates around. Defaults to the midpoint of a limited subsystem. */
	float setpoint = 0;
	bool useSetpoint = false;

	/**
	 * Output driven above and below the setpoint, in the PID's output units.
	 * Zero derives it from the motor gearset and adapts it during the run.
	 */
	float relayAmplitude = 0;

	/**
	 * Error band around the setpoint that the relay ignores, to reject noise.
	 * Zero derives it from the PID threshold and the available travel.
	 */
	float hysteresis = 0;

	/** Stops once consecutive cycles agree within this fraction. */
	float tolerance = 0.1f;

	/** Cycles that must agree before the run is accepted. */
	int agreeingCycles = 3;

	/** Leading cycles discarded while the oscillation settles. */
	int warmupCycles = 2;

	/** Upper bound on cycles, reached only when convergence is slow. */
	int maxCycles = 30;

	/** Gives up after this long. Subsystem::NO_TIMEOUT runs until it converges. */
	std::uint32_t timeout = 15000;

	/** Distance from the setpoint the subsystem may travel. Zero derives it from the limits. */
	float maxExcursion = 0;

	TuningRule rule = TuningRule::ZieglerNicholsPID;
};

struct AutotuneResult {
	bool success = false;
	AutotuneStatus status = AutotuneStatus::InvalidConfig;

	float ultimateGain = 0;
	float ultimatePeriod = 0;

	float kp = 0;
	float ki = 0;
	float kd = 0;

	/** Spread across the accepted cycles, as a fraction of the mean. */
	float periodSpread = 0;
	float amplitudeSpread = 0;

	/** Relay output the run settled on, useful when it was derived. */
	float relayAmplitude = 0;

	int cyclesMeasured = 0;
	std::uint32_t elapsed = 0;

	const char* error = nullptr;
};

/**
 * Finds PID gains for a subsystem by relay feedback.
 *
 * Drives the subsystem with a bang-bang output around the setpoint, measures the
 * amplitude and period of the resulting oscillation, and converts them into
 * gains with the configured tuning rule. The run ends as soon as consecutive
 * cycles agree within the configured tolerance.
 *
 * The subsystem oscillates around the setpoint for the whole run, so give it
 * room to move in both directions and keep clear of it.
 */
class Autotuner {
   public:
	explicit Autotuner(Subsystem& subsystem);

	Autotuner(const Autotuner&) = delete;
	Autotuner& operator=(const Autotuner&) = delete;

	/** Runs a tuning cycle. Blocks until it finishes, fails, or times out. */
	AutotuneResult run(const AutotuneConfig& config);

	/** Runs a tuning cycle and writes the gains into the subsystem's PID. */
	AutotuneResult runAndApply(const AutotuneConfig& config);

	/** Stops an in-progress run from another task. */
	void cancel();

	bool isRunning() const;

	/** Converts a measured gain and period into PID gains. */
	static void computeGains(float ultimateGain, float ultimatePeriod, TuningRule rule, float& kp,
	                         float& ki, float& kd);

   private:
	Subsystem& subsystem;
	std::atomic<bool> cancelRequested;
	std::atomic<bool> running;

	struct Plan {
		float setpoint;
		float excursion;
		float amplitude;
		float hysteresis;
		float amplitudeCeiling;
	};

	bool resolvePlan(const AutotuneConfig& config, Plan& plan, AutotuneResult& result);
};

}  // namespace liftlib
