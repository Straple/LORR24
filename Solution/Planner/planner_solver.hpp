#pragma once

#include "dist_machine.hpp"

static constexpr uint32_t PLANNER_DEPTH = 3;

struct PlannerPosition {
    int x = 0;
    int y = 0;
    int pos = 0;
    int dir = 0;
};

// планирует следующие PLANNER_DEPTH шагов
class PlannerSolver {

    // обозначения:
    // r = robot idx
    // d = planner depth idx

    struct Robot {
        PlannerPosition start;
        int target = -1;
        std::array<Action, PLANNER_DEPTH> actions{};
    };

    std::vector<Robot> robots;

    // map[pos] = true if this pos is free
    // otherwise: false
    std::vector<bool> map;

    uint32_t rows = 0, cols = 0;

public:
    PlannerSolver(uint32_t rows, uint32_t cols, std::vector<bool> map, std::vector<Position> robots_pos, std::vector<int> robots_target);
};
