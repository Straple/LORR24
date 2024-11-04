#pragma once

#include "../randomizer.hpp"
#include "solution_info.hpp"
#include "position.hpp"

#include <thread>

// python3 PlanViz/script/run2.py --map example_problems/random.domain/maps/random-32-32-20.map --plan test.json --end 1000

static constexpr uint32_t PLANNER_DEPTH = 3;

static constexpr uint32_t PLANNING_STEPS = 1'000'000;

#define BUILD_DIST_DP

// планирует следующие PLANNER_DEPTH шагов
class PlannerSolver {

    // обозначения:
    // r = robot idx
    // d = planner depth idx

    using Actions = std::array<Action, PLANNER_DEPTH>;

    struct Robot {
        Position start;
        int target = -1;

        // применяем действие только если оно корректно для статической карты
        // (без учета других роботов)
        // если это действие выходит за карту или переходит в препятствие
        // то мы его не выполняем
        Actions actions{Action::W};
    };

    std::vector<Robot> robots;

    Randomizer rnd;

    double temp = 1;

    std::vector<std::vector<uint32_t>> map_robots_cnt;

    std::vector<std::vector<uint32_t>> map_edge_robots_cnt_gor, map_edge_robots_cnt_ver;

    // pos_to_robot[pos] = robot id or -1
    std::vector<int> pos_to_robot;

    SolutionInfo cur_info;

    /* CHANGE STATE TOOLS*/

    void change_map_robots_cnt(int d, int pos, int val);

    void change_map_edge_robots_cnt(int d, int pos, int to, int val);

    void process_robot_path(uint32_t r, int sign);

    void add_robot_path(uint32_t r);

    void remove_robot_path(uint32_t r);

    /* ALGO TOOLS */

    bool compare(SolutionInfo old, SolutionInfo cur);

    template<typename rollback_t>
    bool consider(SolutionInfo old, rollback_t &&rollback) {
        SolutionInfo cur = get_solution_info();
        if (compare(old, cur)) {
            return true;
        } else {
            rollback();
            return false;
        }
    }

    bool try_change_robot_action();

    bool try_change_robot_path();

    bool try_change_many_robots();

    bool try_move_over();

    void init();

public:
    PlannerSolver() = default;

    PlannerSolver(std::vector<Position> robots_pos, std::vector<int> robots_target);

    void run(std::chrono::steady_clock::time_point end_time, uint64_t random_seed);

    void build_dist();

    [[nodiscard]] std::pair<SolutionInfo, std::vector<Action>> get() const;

    /* SOLUTION INFO */

    [[nodiscard]] SolutionInfo get_solution_info() const;

    [[nodiscard]] double get_x(SolutionInfo info) const;
};
