#pragma once

#include "../randomizer.hpp"
#include "dist_machine.hpp"
#include "solution_info.hpp"

static constexpr uint32_t PLANNER_DEPTH = 3;

struct PlannerPosition {
    int x = 0;
    int y = 0;
    int pos = 0;
    int dir = 0;
};

bool operator<(const PlannerPosition &lhs, const PlannerPosition &rhs);

// планирует следующие PLANNER_DEPTH шагов
class PlannerSolver {

    // обозначения:
    // r = robot idx
    // d = planner depth idx

    struct Robot {
        PlannerPosition start;
        int target = -1;

        // применяем действие только если оно корректно для статической карты
        // (без учета других роботов)
        // если это действие выходит за карту или переходит в препятствие
        // то мы его не выполняем
        std::array<Action, PLANNER_DEPTH> actions{Action::W};
    };

    std::vector<Robot> robots;

    // map[pos] = true if this pos is free
    // otherwise: false
    std::vector<bool> map;

    uint32_t rows = 0, cols = 0;

    Randomizer rnd;

    /* PLANNER POSITION */

    [[nodiscard]] PlannerPosition move_forward(PlannerPosition p) const;

    [[nodiscard]] PlannerPosition rotate(PlannerPosition p) const;

    [[nodiscard]] PlannerPosition counter_rotate(PlannerPosition p) const;

    [[nodiscard]] PlannerPosition simulate_action(PlannerPosition p, Action action) const;

    [[nodiscard]] bool is_valid(const PlannerPosition &p) const;

    /* DIST MACHINE */
    [[nodiscard]] int get_dist(PlannerPosition source, int target) const;

    /* SOLUTION INFO */

    [[nodiscard]] SolutionInfo get_solution_info() const;

public:
    PlannerSolver(uint32_t rows, uint32_t cols, std::vector<bool> map, std::vector<Position> robots_pos,
                  std::vector<int> robots_target, uint64_t random_seed);

    void run(int time_limit);
};
