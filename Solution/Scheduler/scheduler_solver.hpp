#pragma once

#include <Objects/Basic/randomizer.hpp>

#include <SharedEnv.h>

#include <vector>
#include <cstdint>

//-i ./example_problems/warehouse.domain/warehouse_large_5000.json -o test.json -s 1000 -t 1000 -p 100000000

//call(0): 2550, 52.9529s
//call(1): 4374, 66.2482s
//call(2): 5305, 83.7859s
//call(3): 5529, 98.0126s
//call(4): 4682, 142.014s
//call(5): 3702, 221.623s
//total: 26142

class SchedulerSolver {

    double cur_score = 0;

    // desires[r] = task id
    std::vector<uint32_t> desires;

    // task_to_robot[task] = robot id
    std::vector<uint32_t> task_to_robot;

    std::vector<uint32_t> free_robots;

    std::vector<uint32_t> free_tasks;

    // dp[r] = отсортированный вектор (dist, task_id)
    std::vector<std::vector<std::pair<uint32_t, uint32_t>>> dp;

    std::vector<int> timestep_updated;

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

    void triv_solve();

    void solve(TimePoint end_time);

    [[nodiscard]] std::vector<int> get_schedule() const;

    [[nodiscard]] double get_score() const;
};

