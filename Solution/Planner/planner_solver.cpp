#include "planner_solver.hpp"

PlannerSolver::PlannerSolver(uint32_t rows, uint32_t cols, std::vector<bool> map, std::vector<Position> robots_pos, std::vector<int> robots_target) : rows(rows), cols(cols), map(std::move(map)) {
    robots.resize(robots_pos.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        robots[r].start.x = robots_pos[r].x;
        robots[r].start.y = robots_pos[r].y;
        robots[r].start.pos = robots_pos[r].pos;
        robots[r].start.dir = robots_pos[r].dir;
        robots[r].target = robots_target[r];
    }
}
