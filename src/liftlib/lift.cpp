#include "liftlib/lift.hpp"

#include <algorithm>

namespace liftlib {

Lift::Lift(std::initializer_list<Subsystem*> stages) : cancelRequested(false) {
	for (Subsystem* stage : stages) {
		if (stage != nullptr && !hasStage(stage)) {
			this->stages.push_back(stage);
		}
	}
}

Lift::Lift(std::vector<Subsystem*> stages) : cancelRequested(false) {
	for (Subsystem* stage : stages) {
		if (stage != nullptr && !hasStage(stage)) {
			this->stages.push_back(stage);
		}
	}
}

void Lift::initialize() {
	for (Subsystem* stage : stages) {
		stage->initialize();
	}
}

std::vector<std::pair<Subsystem*, float>> Lift::resolve(
    const std::vector<std::pair<Subsystem*, float>>& moves) const {
	std::vector<std::pair<Subsystem*, float>> resolved;
	for (const std::pair<Subsystem*, float>& move : moves) {
		if (move.first == nullptr || !hasStage(move.first)) {
			continue;
		}

		auto existing = std::find_if(
		    resolved.begin(), resolved.end(),
		    [&move](const std::pair<Subsystem*, float>& kept) { return kept.first == move.first; });

		if (existing != resolved.end()) {
			existing->second = move.second;
		} else {
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
	for (const std::pair<Subsystem*, float>& move : moves) {
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
	for (const std::pair<Subsystem*, float>& move : moves) {
		if (move.first == nullptr || !move.first->isSettled()) {
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

void Lift::stopStages() {
	for (Subsystem* stage : stages) {
		stage->stop();
	}
}

void Lift::moveToBlocking(const std::vector<float>& targets, std::uint32_t timeout) {
	reset();

	std::uint32_t start = pros::millis();
	std::uint32_t now = start;

	while (!isSettled() && !cancelRequested.load()) {
		if (timeout != NO_TIMEOUT && pros::millis() - start >= timeout) {
			break;
		}
		update(targets);
		pros::Task::delay_until(&now, Subsystem::LOOP_DELAY_MS);
	}

	brake();
}

bool Lift::moveTo(const std::vector<float>& targets, bool async, std::uint32_t timeout) {
	if (targets.size() < stages.size()) {
		return false;
	}

	cancelTask();
	stopStages();

	std::vector<float> owned(targets.begin(), targets.begin() + stages.size());

	if (!async) {
		cancelRequested.store(false);
		moveToBlocking(owned, timeout);
		return true;
	}

	cancelRequested.store(false);
	task = std::make_unique<pros::Task>(
	    [this, owned, timeout]() { moveToBlocking(owned, timeout); });
	return true;
}

void Lift::moveToBlocking(const std::vector<std::pair<Subsystem*, float>>& moves,
                          std::uint32_t timeout) {
	for (const std::pair<Subsystem*, float>& move : moves) {
		move.first->reset();
	}

	std::uint32_t start = pros::millis();
	std::uint32_t now = start;

	while (!isSettled(moves) && !cancelRequested.load()) {
		if (timeout != NO_TIMEOUT && pros::millis() - start >= timeout) {
			break;
		}
		update(moves);
		pros::Task::delay_until(&now, Subsystem::LOOP_DELAY_MS);
	}

	for (const std::pair<Subsystem*, float>& move : moves) {
		move.first->brake();
	}
}

bool Lift::moveTo(std::initializer_list<std::pair<Subsystem*, float>> moves, bool async,
                  std::uint32_t timeout) {
	std::vector<std::pair<Subsystem*, float>> resolved = resolve(moves);
	if (resolved.empty()) {
		return false;
	}

	cancelTask();
	for (const std::pair<Subsystem*, float>& move : resolved) {
		move.first->stop();
	}

	if (!async) {
		cancelRequested.store(false);
		moveToBlocking(resolved, timeout);
		return true;
	}

	cancelRequested.store(false);
	task = std::make_unique<pros::Task>(
	    [this, resolved, timeout]() { moveToBlocking(resolved, timeout); });
	return true;
}

void Lift::cancelTask() {
	if (task == nullptr) {
		return;
	}

	cancelRequested.store(true);
	task->join();
	task.reset();
	cancelRequested.store(false);
}

bool Lift::waitUntilSettled(std::uint32_t timeout) {
	if (task == nullptr) {
		return true;
	}

	if (timeout == NO_TIMEOUT) {
		task->join();
		task.reset();
		return true;
	}

	std::uint32_t start = pros::millis();
	std::uint32_t now = start;
	while (isMoving()) {
		if (pros::millis() - start >= timeout) {
			stop();
			return false;
		}
		pros::Task::delay_until(&now, Subsystem::LOOP_DELAY_MS);
	}

	task->join();
	task.reset();
	return true;
}

void Lift::stop() {
	cancelTask();
	stopStages();
	brake();
}

bool Lift::isMoving() const {
	return task != nullptr && task->get_state() != pros::E_TASK_STATE_DELETED;
}

Lift::~Lift() {
	cancelTask();
}

bool Lift::hasStage(Subsystem* stage) const {
	return std::find(stages.begin(), stages.end(), stage) != stages.end();
}

bool Lift::moveStageTo(std::size_t index, float target, bool async, std::uint32_t timeout) {
	if (index >= stages.size()) {
		return false;
	}

	cancelTask();
	stages[index]->moveTo(target, async, timeout);
	return true;
}

Subsystem* Lift::getStage(std::size_t index) const {
	return index < stages.size() ? stages[index] : nullptr;
}

std::size_t Lift::stageCount() const {
	return stages.size();
}

}  // namespace liftlib
