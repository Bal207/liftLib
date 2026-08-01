#pragma once

#include <initializer_list>
#include <utility>
#include <vector>

#include "liftlib/subsystem.hpp"

class Lift {
   private:
	std::vector<Subsystem*> stages;

	std::vector<std::pair<Subsystem*, float>> resolve(
	    const std::vector<std::pair<Subsystem*, float>>& moves) const;

   public:
	Lift(std::initializer_list<Subsystem*> stages);

	explicit Lift(std::vector<Subsystem*> stages);

	void initialize();

	void update(const std::vector<float>& targets);

	void update(const std::vector<std::pair<Subsystem*, float>>& moves);

	bool isSettled() const;

	bool isSettled(const std::vector<std::pair<Subsystem*, float>>& moves) const;

	void reset();

	void brake();

	void moveTo(const std::vector<float>& targets);

	void moveTo(std::initializer_list<std::pair<Subsystem*, float>> moves);

	void moveStageTo(std::size_t index, float target);

	bool hasStage(Subsystem* stage) const;

	Subsystem* getStage(std::size_t index) const;

	std::size_t stageCount() const;
};
