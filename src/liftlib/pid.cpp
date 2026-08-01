#include "liftlib/pid.hpp"

#include <algorithm>
#include <cmath>

PID::PID(float kp, float ki, float kd, float threshold)
    : KP(kp), KI(ki), KD(kd), threshold(threshold), slew(0), previousOutput(0) {}

PID::PID(float kp, float ki, float kd, float threshold, float slew)
    : KP(kp), KI(ki), KD(kd), threshold(threshold), slew(std::abs(slew)), previousOutput(0) {}

float PID::calculate(float setpoint, float actual) {
	return calculate(setpoint - actual);
}

float PID::calculate(float error) {
	float derivative = error - previousError;
	integral = integral + error;
	float output = KP * error + KI * integral + KD * derivative;
	previousError = error;

	if (slew > 0) {
		output = std::clamp(output, previousOutput - slew, previousOutput + slew);
	}

	previousOutput = output;
	return output;
}

void PID::reset() {
	integral = 0;
	previousError = 0;
	previousOutput = 0;
}

bool PID::isSettled(float error) {
	return std::abs(error) < threshold;
}

void PID::setSlew(float slew) {
	this->slew = std::abs(slew);
}

float PID::getSlew() const {
	return slew;
}
