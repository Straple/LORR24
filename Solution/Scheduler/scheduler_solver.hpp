#pragma once

#include <Objects/Basic/randomizer.hpp>
#include <settings.hpp>

#include <SharedEnv.h>

#include <vector>
#include <cstdint>

class SchedulerSolver {

    double cur_score = 0;

    std::vector<uint32_t> old_desires;

    // desires[r] = task id
    std::vector<uint32_t> desires;

    // task_to_robot[task] = robot id
    std::vector<uint32_t> task_to_robot;

    std::vector<uint32_t> free_robots;

    std::vector<uint32_t> free_tasks;

    // dp[r] = отсортированный вектор (dist, task_id)
    std::vector<std::vector<std::pair<uint32_t, uint32_t>>> dp;

    std::vector<int> timestep_updated;

    // timestep_changed_task[r] = когда была изменена задача у робота r
    std::vector<int> timestep_changed_task;

    SharedEnvironment *env = nullptr;

    double temp = 1;

    void rebuild_dp(uint32_t r);

    [[nodiscard]] bool compare(double cur_score, double old_score, Randomizer &rnd) const;

    template<typename rollback_t>
    bool consider(double old_score, Randomizer &rnd, rollback_t &&rollback) {
        if (compare(cur_score, old_score, rnd)) {
            return true;
        } else {
            rollback();
            ASSERT(std::abs(old_score - cur_score) < 1e-9, "invalid rollback");
            return false;
        }
    }

    [[nodiscard]] uint32_t get_dist(uint32_t r, uint32_t t) const;

    void set(uint32_t r, uint32_t t);

    bool try_peek_task(Randomizer &rnd);

    bool try_smart(Randomizer &rnd);

    void validate();

public:

    SchedulerSolver() = default;

    explicit SchedulerSolver(SharedEnvironment *env);

    void update();

    void rebuild_dp(TimePoint end_time);

    void triv_solve(TimePoint end_time);

    void solve(TimePoint end_time);

    [[nodiscard]] std::vector<int> get_schedule() const;

    [[nodiscard]] double get_score() const;
};

