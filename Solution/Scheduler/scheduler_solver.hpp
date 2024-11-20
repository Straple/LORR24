#pragma once

#include <SharedEnv.h>
#include <Tasks.h>
#include <heuristics.h>

#include <Objects/Basic/randomizer.hpp>
#include <Objects/Environment/graph.hpp>
#include <Objects/Environment/heuristic_matrix.hpp>

// их 1763
// 1817

// их расстояния, мой солвер 1856

class SchedulerSolver {

    static constexpr inline uint32_t MAX_SCORE = 1e9;

    struct Robot {
        uint32_t id = 0;

        uint32_t node = 0;

        // индекс в tasks задачи, которую мы взяли
        uint32_t task_id = -1;

        uint64_t score = MAX_SCORE;
    };

    struct Task {
        uint32_t id = 0;

        std::vector<uint32_t> path;// position coords

        // индекс в robots, который взял эту задачу
        uint32_t robot_id = -1;
    };

    std::vector<Robot> robots;

    std::vector<Task> tasks;

    uint64_t total_score = 0;

    SharedEnvironment *env_ptr = nullptr;

    double temp = 1;

    bool compare(uint64_t old, uint64_t cur, Randomizer &rnd);

    template<typename rollback_t>
    bool consider(uint64_t old, Randomizer &rnd, rollback_t &&rollback) {
        if (compare(old, total_score, rnd)) {
            return true;
        } else {
            rollback();
            //std::cout << total_score << ' ' << old << std::endl;
            ASSERT(total_score == old, "invalid rollback");
            return false;
        }
    }

    [[nodiscard]] uint64_t get_score(uint32_t r, uint32_t t) const;

    void remove_task(uint32_t r);

    void add_task(uint32_t r, uint32_t t);

    bool try_change(Randomizer &rnd);

    [[nodiscard]] uint64_t get_h(uint32_t source, uint32_t dest) const;

public:
    SchedulerSolver() = default;

    void solve(SharedEnvironment &env, uint64_t seed, const TimePoint end_time);

    void set_schedule(std::vector<int> &proposed_schedule) const;

    uint64_t get_score() const;
};
