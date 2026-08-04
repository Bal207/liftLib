#include "liftlib/autotuner.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <numbers>

#include "api.h"

namespace liftlib {

namespace {

constexpr std::uint32_t MIN_HALF_PERIOD_MS = 40;
constexpr float MIN_AMPLITUDE = 1e-3f;
constexpr float EXCURSION_MARGIN = 0.9f;
constexpr float DEFAULT_OUTPUT_LIMIT = 200.0f;
constexpr float DEFAULT_HYSTERESIS = 1.0f;
constexpr float MIN_HYSTERESIS = 0.25f;
constexpr float HYSTERESIS_FRACTION = 0.1f;
constexpr float MIN_EXCURSION_RATIO = 4.0f;
constexpr float START_AMPLITUDE_FRACTION = 0.3f;
constexpr float AMPLITUDE_STEP_UP = 1.6f;
constexpr float AMPLITUDE_STEP_DOWN = 0.6f;
constexpr float MIN_AMPLITUDE_FRACTION = 0.05f;
constexpr float SWING_TARGET_FRACTION = 0.25f;
constexpr float SWING_CEILING_FRACTION = 0.75f;

struct Cycle {
	float period;
	float amplitude;
};

float spread(const std::deque<Cycle>& cycles, bool periods) {
	if (cycles.size() < 2) {
		return 1.0f;
	}

	float low = periods ? cycles.front().period : cycles.front().amplitude;
	float high = low;
	float total = 0;

	for (const Cycle& cycle : cycles) {
		float value = periods ? cycle.period : cycle.amplitude;
		low = std::min(low, value);
		high = std::max(high, value);
		total += value;
	}

	float mean = total / cycles.size();
	return mean > MIN_AMPLITUDE ? (high - low) / mean : 1.0f;
}

float mean(const std::deque<Cycle>& cycles, bool periods) {
	if (cycles.empty()) {
		return 0;
	}

	float total = 0;
	for (const Cycle& cycle : cycles) {
		total += periods ? cycle.period : cycle.amplitude;
	}
	return total / cycles.size();
}

} 

Autotuner::Autotuner(Subsystem& subsystem)
    : subsystem(subsystem), cancelRequested(false), running(false) {}

void Autotuner::computeGains(float ultimateGain, float ultimatePeriod, TuningRule rule, float& kp,
                             float& ki, float& kd) {
	const float ku = ultimateGain;
	const float tu = ultimatePeriod;

	if (ku <= 0 || tu <= 0) {
		kp = 0;
		ki = 0;
		kd = 0;
		return;
	}

	float ti = 0;
	float td = 0;

	switch (rule) {
		case TuningRule::ZieglerNicholsP:
			kp = 0.5f * ku;
			break;

		case TuningRule::ZieglerNicholsPI:
			kp = 0.45f * ku;
			ti = tu / 1.2f;
			break;

		case TuningRule::ZieglerNicholsPD:
			kp = 0.8f * ku;
			td = tu / 8.0f;
			break;

		case TuningRule::PessenIntegral:
			kp = 0.7f * ku;
			ti = tu / 2.5f;
			td = 3.0f * tu / 20.0f;
			break;

		case TuningRule::SomeOvershoot:
			kp = ku / 3.0f;
			ti = tu / 2.0f;
			td = tu / 3.0f;
			break;

		case TuningRule::NoOvershoot:
			kp = 0.2f * ku;
			ti = tu / 2.0f;
			td = tu / 3.0f;
			break;

		case TuningRule::ZieglerNicholsPID:
		default:
			kp = 0.6f * ku;
			ti = tu / 2.0f;
			td = tu / 8.0f;
			break;
	}

	const float dt = static_cast<float>(Subsystem::LOOP_DELAY_MS) / 1000.0f;

	ki = ti > 0 ? (kp / ti) * dt : 0;
	kd = td > 0 ? (kp * td) / dt : 0;
}

bool Autotuner::resolvePlan(const AutotuneConfig& config, Plan& plan, AutotuneResult& result) {
	const bool limited = subsystem.hasLimits();
	const float low = subsystem.getMinPosition();
	const float high = subsystem.getMaxPosition();

	if (config.useSetpoint) {
		plan.setpoint = limited ? std::clamp(config.setpoint, low, high) : config.setpoint;
	} else if (limited) {
		plan.setpoint = (low + high) / 2.0f;
	} else {
		plan.setpoint = subsystem.getPosition();
	}

	const float room =
	    limited ? std::min(plan.setpoint - low, high - plan.setpoint) : 0.0f;

	if (config.maxExcursion > 0) {
		plan.excursion = config.maxExcursion;
	} else if (limited) {
		plan.excursion = room * EXCURSION_MARGIN;
	} else {
		plan.excursion = 0;
	}

	if (limited) {
		plan.excursion = std::min(plan.excursion, room);
	}

	if (config.hysteresis > 0) {
		plan.hysteresis = config.hysteresis;
	} else {
		const float threshold = subsystem.getPID().getThreshold();
		plan.hysteresis = threshold > 0 ? threshold : DEFAULT_HYSTERESIS;

		if (plan.excursion > 0) {
			plan.hysteresis = std::min(plan.hysteresis, plan.excursion * HYSTERESIS_FRACTION);
		}
		plan.hysteresis = std::max(plan.hysteresis, MIN_HYSTERESIS);
	}
	if (limited && plan.excursion < plan.hysteresis * MIN_EXCURSION_RATIO) {
		result.status = AutotuneStatus::OutOfRoom;
		result.error = "not enough travel between the limits to oscillate";
		return false;
	}

	const float outputLimit = subsystem.getPID().getMaxOutput();
	plan.amplitudeCeiling = outputLimit > 0 ? outputLimit : DEFAULT_OUTPUT_LIMIT;

	// The relay swings about the holding output, so leave room for the bias or
	// one side of the swing would clip against the output limit.
	plan.amplitudeCeiling =
	    std::max(plan.amplitudeCeiling - std::abs(subsystem.holdOutput()), MIN_HYSTERESIS);

	if (config.relayAmplitude > 0) {
		plan.amplitude = std::min(config.relayAmplitude, plan.amplitudeCeiling);
	} else {
		plan.amplitude = plan.amplitudeCeiling * START_AMPLITUDE_FRACTION;
	}

	return true;
}

AutotuneResult Autotuner::run(const AutotuneConfig& config) {
	AutotuneResult result;

	if (config.relayAmplitude < 0) {
		result.error = "relayAmplitude must not be negative";
		return result;
	}

	if (config.hysteresis < 0) {
		result.error = "hysteresis must not be negative";
		return result;
	}

	if (config.tolerance <= 0) {
		result.error = "tolerance must be positive";
		return result;
	}

	if (config.agreeingCycles < 2) {
		result.error = "agreeingCycles must be at least 2";
		return result;
	}

	if (config.warmupCycles < 0) {
		result.error = "warmupCycles must not be negative";
		return result;
	}

	if (config.maxCycles <= config.warmupCycles + config.agreeingCycles) {
		result.error = "maxCycles must exceed warmupCycles plus agreeingCycles";
		return result;
	}

	Plan plan{};
	if (!resolvePlan(config, plan, result)) {
		return result;
	}

	const float setpoint = plan.setpoint;
	const float excursion = plan.excursion;
	const float hysteresis = plan.hysteresis;
	const bool adaptAmplitude = config.relayAmplitude <= 0;
	const float minAmplitude = plan.amplitudeCeiling * MIN_AMPLITUDE_FRACTION;
	float amplitude = plan.amplitude;

	running.store(true);

	const std::uint32_t start = pros::millis();
	std::uint32_t now = start;

	float position = subsystem.getPosition();
	bool positive = position < setpoint;
	int flips = 0;
	int cyclesSeen = 0;

	float peak = position;
	float trough = position;
	std::uint32_t lastFlip = start;
	float halfPeriod = 0;

	std::deque<Cycle> window;
	AutotuneStatus status = AutotuneStatus::TimedOut;
	result.error = "timed out before the oscillation converged";

	while (true) {
		if (cancelRequested.load()) {
			status = AutotuneStatus::Cancelled;
			result.error = "cancelled";
			break;
		}

		if (config.timeout != Subsystem::NO_TIMEOUT &&
		    pros::millis() - start >= config.timeout) {
			break;
		}

		position = subsystem.getPosition();
		const float error = setpoint - position;

		if (excursion > 0 && std::abs(error) > excursion) {
			if (subsystem.hasLimits() || flips > 1) {
				status = AutotuneStatus::ExcursionExceeded;
				result.error = "travelled past the allowed excursion";
				break;
			}
		}

		peak = std::max(peak, position);
		trough = std::min(trough, position);

		const bool wantPositive = positive ? error > -hysteresis : error > hysteresis;

		if (wantPositive != positive) {
			const std::uint32_t sinceFlip = pros::millis() - lastFlip;
			if (sinceFlip >= MIN_HALF_PERIOD_MS) {
				if (flips > 0 && flips % 2 == 0) {
					const float period = (halfPeriod + static_cast<float>(sinceFlip)) / 1000.0f;
					const float swing = (peak - trough) / 2.0f;

					if (adaptAmplitude) {
						const float reference = excursion > 0 ? excursion : hysteresis * 10.0f;
						float scaled = amplitude;

						if (swing < reference * SWING_TARGET_FRACTION) {
							scaled = amplitude * AMPLITUDE_STEP_UP;
						} else if (swing > reference * SWING_CEILING_FRACTION) {
							scaled = amplitude * AMPLITUDE_STEP_DOWN;
						}

						scaled = std::clamp(scaled, minAmplitude, plan.amplitudeCeiling);
						if (scaled != amplitude) {
							amplitude = scaled;
							window.clear();
						}
					}

					cyclesSeen++;
					if (cyclesSeen > config.warmupCycles && swing > MIN_AMPLITUDE) {
						window.push_back({period, swing});
						while (static_cast<int>(window.size()) > config.agreeingCycles) {
							window.pop_front();
						}

						if (static_cast<int>(window.size()) == config.agreeingCycles &&
						    spread(window, true) <= config.tolerance &&
						    spread(window, false) <= config.tolerance) {
							status = AutotuneStatus::Success;
							result.error = nullptr;
							break;
						}
					}

					if (cyclesSeen >= config.maxCycles) {
						if (static_cast<int>(window.size()) >= 2) {
							status = AutotuneStatus::Success;
							result.error = nullptr;
						} else {
							status = AutotuneStatus::NoOscillation;
							result.error = "no usable oscillation within maxCycles";
						}
						break;
					}

					peak = position;
					trough = position;
				}

				halfPeriod = static_cast<float>(sinceFlip);
				lastFlip = pros::millis();
				flips++;
				positive = wantPositive;
			}
		}

		// Swing about the holding output so a gravity-loaded mechanism rises and
		// falls evenly instead of sagging through the whole run.
		const float bias = subsystem.holdOutput();
		subsystem.setOutput(bias + (positive ? amplitude : -amplitude));
		pros::Task::delay_until(&now, Subsystem::LOOP_DELAY_MS);
	}

	subsystem.brake();
	running.store(false);
	cancelRequested.store(false);

	result.status = status;
	result.cyclesMeasured = cyclesSeen;
	result.elapsed = pros::millis() - start;

	if (status != AutotuneStatus::Success || window.empty()) {
		if (status == AutotuneStatus::Success) {
			result.status = AutotuneStatus::NoOscillation;
			result.error = "no usable oscillation was measured";
		}
		return result;
	}

	const float swing = mean(window, false);
	const float period = mean(window, true);

	if (swing <= MIN_AMPLITUDE || period <= 0) {
		result.status = AutotuneStatus::NoOscillation;
		result.error = "measured oscillation was too small to use";
		return result;
	}

	result.periodSpread = spread(window, true);
	result.amplitudeSpread = spread(window, false);
	result.relayAmplitude = amplitude;
	result.ultimateGain = (4.0f * amplitude) / (std::numbers::pi_v<float> * swing);
	result.ultimatePeriod = period;

	computeGains(result.ultimateGain, result.ultimatePeriod, config.rule, result.kp, result.ki,
	             result.kd);

	result.success = true;
	return result;
}

AutotuneResult Autotuner::runAndApply(const AutotuneConfig& config) {
	AutotuneResult result = run(config);

	if (result.success) {
		subsystem.getPID().setGains(result.kp, result.ki, result.kd);
	}

	return result;
}

void Autotuner::cancel() {
	cancelRequested.store(true);
}

bool Autotuner::isRunning() const {
	return running.load();
}

} 
