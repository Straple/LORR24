#pragma once

#include "dist_machine.hpp"

static constexpr uint32_t PLANNER_DEPTH = 3;

// планирует следующие PLANNER_DEPTH шагов
class PlannerSolver {

    // обозначения:
    // r = robot idx
    // d = planner depth idx

    struct Robot {
        Position start;
        Position target;
        std::array<Action, PLANNER_DEPTH> actions{};
    };

    std::vector<Robot> robots;

    // map[pos] = true if this pos is free
    // otherwise: false
    std::vector<bool> map;

    uint32_t rows = 0, cols = 0;

public:
    PlannerSolver(uint32_t rows, uint32_t cols, std::vector<bool> map, std::vector<Position> robots_pos, std::vector<Position> robots_target);
};
