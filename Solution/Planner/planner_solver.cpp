#include "planner_solver.hpp"

PlannerSolver::PlannerSolver(uint32_t rows, uint32_t cols, std::vector<bool> map, std::vector<Position> robots_pos, std::vector<Position> robots_target) : rows(rows), cols(cols), map(std::move(map)) {
    robots.resize(robots_pos.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        robots[r].start = robots_pos[r];
        robots[r].target = robots_target[r];
    }
}
