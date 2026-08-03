#pragma once

#include <atomic>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <utility>
#include <vector>

#include "api.h"
#include "liftlib/subsystem.hpp"

class Lift {
   private:
	std::vector<Subsystem*> stages;

	std::vector<std::pair<Subsystem*, float>> resolve(
	    const std::vector<std::pair<Subsystem*, float>>& moves) const;

	std::unique_ptr<pros::Task> task;
	std::atomic<bool> cancelRequested;

	void cancelTask();

	void stopStages();

	void moveToBlocking(const std::vector<float>& targets, std::uint32_t timeout);

	void moveToBlocking(const std::vector<std::pair<Subsystem*, float>>& moves,
	                    std::uint32_t timeout);

   public:
	static constexpr std::uint32_t NO_TIMEOUT = Subsystem::NO_TIMEOUT;

	Lift(std::initializer_list<Subsystem*> stages);

	explicit Lift(std::vector<Subsystem*> stages);

	Lift(const Lift&) = delete;
	Lift& operator=(const Lift&) = delete;

	~Lift();

	void initialize();

	void update(const std::vector<float>& targets);

	void update(const std::vector<std::pair<Subsystem*, float>>& moves);

	bool isSettled() const;

	bool isSettled(const std::vector<std::pair<Subsystem*, float>>& moves) const;

	void reset();

	void brake();

	/**
	 * Moves every stage to its corresponding target.
	 *
	 * When async is true (the default) the motion runs in its own task and this
	 * call returns immediately. When async is false the call blocks until every
	 * stage settles or the timeout elapses.
	 *
	 * Returns false when the target count does not cover every stage.
	 *
	 * Starting a new move cancels any async move already in progress.
	 */
	bool moveTo(const std::vector<float>& targets, bool async = true,
	            std::uint32_t timeout = NO_TIMEOUT);

	/**
	 * Moves the named stages to their targets, leaving other stages untouched.
	 *
	 * When async is true (the default) the motion runs in its own task and this
	 * call returns immediately. When async is false the call blocks until the
	 * named stages settle or the timeout elapses.
	 *
	 * Returns false when no named stage belongs to this lift.
	 *
	 * Starting a new move cancels any async move already in progress.
	 */
	bool moveTo(std::initializer_list<std::pair<Subsystem*, float>> moves, bool async = true,
	            std::uint32_t timeout = NO_TIMEOUT);

	bool moveStageTo(std::size_t index, float target, bool async = true,
	                 std::uint32_t timeout = NO_TIMEOUT);

	/**
	 * Blocks until an in-progress async move finishes.
	 *
	 * Returns false when the timeout elapsed first, in which case the move is
	 * cancelled and every stage is braked. A timeout of NO_TIMEOUT waits
	 * indefinitely.
	 */
	bool waitUntilSettled(std::uint32_t timeout = NO_TIMEOUT);

	/** Stops an in-progress async move and brakes every stage. */
	void stop();

	/** True while an async move is still running. */
	bool isMoving() const;

	bool hasStage(Subsystem* stage) const;

	Subsystem* getStage(std::size_t index) const;

	std::size_t stageCount() const;
};
