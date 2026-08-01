#pragma once

#include <vector>

#include "liftlib/motor_config.hpp"
#include "liftlib/pid.hpp"

class Subsystem {
   protected:
	std::vector<MotorConfig> motorConfigs;
	PID pid;

	float minPosition;
	float maxPosition;

	bool limited;

   public:
	Subsystem(std::vector<MotorConfig> motorConfigs, PID pid);

	Subsystem(std::vector<MotorConfig> motorConfigs, PID pid, float minPosition, float maxPosition);

	virtual ~Subsystem() = default;

	virtual void initialize();

	virtual float getPosition() const;

	float clampTarget(float target) const;

	virtual void update(float target);

	virtual bool isSettled() const;

	virtual void reset();

	virtual void brake();

	virtual void moveTo(float target);

   private:
	float lastError;
	bool settled;
};
