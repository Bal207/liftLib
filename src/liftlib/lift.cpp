#include "liftlib/lift.hpp"

#include <algorithm>

Lift::Lift(std::initializer_list<Subsystem*> stages) : stages(stages) {}

Lift::Lift(std::vector<Subsystem*> stages) : stages(stages) {}

void Lift::initialize() {
	for (Subsystem* stage : stages) {
		stage->initialize();
	}
}

std::vector<std::pair<Subsystem*, float>> Lift::resolve(
    const std::vector<std::pair<Subsystem*, float>>& moves) const {
	std::vector<std::pair<Subsystem*, float>> resolved;
	for (const std::pair<Subsystem*, float>& move : moves) {
		if (move.first != nullptr && hasStage(move.first)) {
			resolved.push_back(move);
		}
	}
	return resolved;
}

void Lift::update(const std::vector<float>& targets) {
	std::size_t count = std::min(stages.size(), targets.size());
	for (std::size_t i = 0; i < count; i++) {
		stages[i]->update(targets[i]);
	}
}

void Lift::update(const std::vector<std::pair<Subsystem*, float>>& moves) {
	for (const std::pair<Subsystem*, float>& move : resolve(moves)) {
		move.first->update(move.second);
	}
}

bool Lift::isSettled() const {
	for (const Subsystem* stage : stages) {
		if (!stage->isSettled()) {
			return false;
		}
	}
	return true;
}

bool Lift::isSettled(const std::vector<std::pair<Subsystem*, float>>& moves) const {
	for (const std::pair<Subsystem*, float>& move : resolve(moves)) {
		if (!move.first->isSettled()) {
			return false;
		}
	}
	return true;
}

void Lift::reset() {
	for (Subsystem* stage : stages) {
		stage->reset();
	}
}

void Lift::brake() {
	for (Subsystem* stage : stages) {
		stage->brake();
	}
}

void Lift::moveTo(const std::vector<float>& targets) {
	if (targets.size() < stages.size()) {
		return;
	}

	reset();
	while (!isSettled()) {
		update(targets);
		pros::delay(20);
	}
	brake();
}

void Lift::moveTo(std::initializer_list<std::pair<Subsystem*, float>> moves) {
	std::vector<std::pair<Subsystem*, float>> resolved = resolve(moves);
	if (resolved.empty()) {
		return;
	}

	for (const std::pair<Subsystem*, float>& move : resolved) {
		move.first->reset();
	}

	while (!isSettled(resolved)) {
		update(resolved);
		pros::delay(20);
	}

	for (const std::pair<Subsystem*, float>& move : resolved) {
		move.first->brake();
	}
}

bool Lift::hasStage(Subsystem* stage) const {
	return std::find(stages.begin(), stages.end(), stage) != stages.end();
}

void Lift::moveStageTo(std::size_t index, float target) {
	if (index < stages.size()) {
		stages[index]->moveTo(target);
	}
}

Subsystem* Lift::getStage(std::size_t index) const {
	return index < stages.size() ? stages[index] : nullptr;
}

std::size_t Lift::stageCount() const {
	return stages.size();
}
