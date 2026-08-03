#pragma once

class PID {
   private:
	float KP = 0, KI = 0, KD = 0;
	float previousError = 0;
	float integral = 0;
	float threshold = 0;

	float slew = 0;
	float previousOutput = 0;
	bool hasPreviousError = false;

	float maxIntegral = 0;
	float integralZone = 0;
	float maxOutput = 0;

   public:
	PID(float kp, float ki, float kd, float threshold, float slew = 0);

	float calculate(float setpoint, float actual);
	float calculate(float error);

	void reset();

	bool isSettled(float error) const;

	void setSlew(float slew);
	float getSlew() const;

	void setMaxIntegral(float maxIntegral);
	float getMaxIntegral() const;

	void setIntegralZone(float integralZone);
	float getIntegralZone() const;

	void setMaxOutput(float maxOutput);
	float getMaxOutput() const;

	float getThreshold() const;
};
