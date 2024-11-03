#pragma once

#include "../randomizer.hpp"
#include "dist_machine.hpp"
#include "solution_info.hpp"

#include <thread>

// python3 PlanViz/script/run2.py --map example_problems/random.domain/maps/random-32-32-20.map --plan test.json --end 1000

static constexpr uint32_t PLANNER_DEPTH = 3;

static constexpr uint32_t PLANNING_STEPS = 500'000;

static constexpr uint32_t THREADS = 8;

struct PlannerPosition {
    int x = 0;
    int y = 0;
    int pos = 0;
    int dir = 0;
};

bool operator<(const PlannerPosition &lhs, const PlannerPosition &rhs);

bool operator==(const PlannerPosition &lhs, const PlannerPosition &rhs);

bool operator!=(const PlannerPosition &lhs, const PlannerPosition &rhs);

// планирует следующие PLANNER_DEPTH шагов
class PlannerSolver {

    // обозначения:
    // r = robot idx
    // d = planner depth idx

    using Actions = std::array<Action, PLANNER_DEPTH>;

    struct Robot {
        PlannerPosition start;
        int target = -1;

        // применяем действие только если оно корректно для статической карты
        // (без учета других роботов)
        // если это действие выходит за карту или переходит в препятствие
        // то мы его не выполняем
        Actions actions{Action::W};
    };

    std::vector<Robot> robots;

    uint32_t rows = 0, cols = 0;

    // map[pos] = true if this pos is free
    // otherwise: false
    std::vector<bool> map;

    // pos_to_robot[pos] = robot id or -1
    std::vector<int> pos_to_robot;

    Randomizer rnd;

    double temp = 1;

    // dist_dp[target][source][dir]
    static inline std::vector<std::vector<std::vector<int> > > dist_dp;

    std::vector<std::vector<uint32_t>> map_robots_cnt;

    std::vector<std::vector<uint32_t>> map_edge_robots_cnt_gor, map_edge_robots_cnt_ver;

    SolutionInfo cur_info;

    /* PLANNER POSITION */

    [[nodiscard]] PlannerPosition move_forward(PlannerPosition p) const;

    [[nodiscard]] PlannerPosition rotate(PlannerPosition p) const;

    [[nodiscard]] PlannerPosition counter_rotate(PlannerPosition p) const;

    [[nodiscard]] PlannerPosition simulate_action(PlannerPosition p, Action action) const;

    // корректная позиция и там нет препятствия
    [[nodiscard]] bool is_valid(const PlannerPosition &p) const;

    /* DIST MACHINE */

    [[nodiscard]] int get_dist(PlannerPosition source, int target) const;

    void build_dist(int target);

    /* SOLUTION INFO */

    [[nodiscard]] SolutionInfo get_solution_info() const;

    /* CHANGE STATE TOOLS*/

    void change_map_robots_cnt(int d, int pos, int val);

    void change_map_edge_robots_cnt(int d, int pos, int to, int val);

    void process_robot_path(uint32_t r, int sign);

    void add_robot_path(uint32_t r);

    void remove_robot_path(uint32_t r);

    /* ALGO TOOLS */

    double get_x(SolutionInfo info);

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
    PlannerSolver(uint32_t rows, uint32_t cols, std::vector<bool> map, std::vector<Position> robots_pos,
                  std::vector<int> robots_target, uint64_t random_seed);

    void run(int time_limit);

    [[nodiscard]] std::pair<SolutionInfo, std::vector<Action>> get() const;

    void build_dist();
};
