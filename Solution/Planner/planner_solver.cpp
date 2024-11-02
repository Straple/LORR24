#include "planner_solver.hpp"

bool operator<(const PlannerPosition &lhs, const PlannerPosition &rhs) {
    if (lhs.pos != rhs.pos) {
        return lhs.pos < rhs.pos;
    }
    return lhs.dir < rhs.dir;
}

PlannerPosition PlannerSolver::move_forward(PlannerPosition p) const {
    if (p.dir == 0) {
        p.y++;
        p.pos++;
    } else if (p.dir == 1) {
        p.pos += static_cast<int>(cols);
        p.x++;
    } else if (p.dir == 2) {
        p.pos--;
        p.y--;
    } else if (p.dir == 3) {
        p.pos -= static_cast<int>(cols);
        p.x--;
    } else {
        FAILED_ASSERT("invalid position dir: " + std::to_string(p.dir));
    }
    return p;
}

PlannerPosition PlannerSolver::rotate(PlannerPosition p) const {
    p.dir = (p.dir + 1) % 4;
    return p;
}

PlannerPosition PlannerSolver::counter_rotate(PlannerPosition p) const {
    p.dir = (p.dir - 1 + 4) % 4;
    return p;
}

PlannerPosition PlannerSolver::simulate_action(PlannerPosition p, Action action) const {
    if (action == Action::FW) {
        return move_forward(p);
    } else if (action == Action::CR) {
        return rotate(p);
    } else if (action == Action::CCR) {
        return counter_rotate(p);
    } else if (action == Action::W) {
        return p;
    } else {
        ASSERT(false, "unexpected action");
        return p;
    }
}

bool PlannerSolver::is_valid(const PlannerPosition &p) const {
    return 0 <= p.x && p.x < rows &&//
           0 <= p.y && p.y < cols;
}

int PlannerSolver::get_dist(PlannerPosition source, int target) const {
    vector<PlannerPosition> Q0, Q1;
    Q0.push_back(source);

    std::set<PlannerPosition> visited;
    visited.insert(source);

    int d = 0;
    while (!Q0.empty() || !Q1.empty()) {
        if (Q0.empty()) {
            std::swap(Q0, Q1);
            d++;
        }

        PlannerPosition p = Q0.back();
        Q0.pop_back();

        ASSERT(is_valid(p), "p is invalid");

        if (p.pos == target) {
            return d;
        }

        visited.insert(p);

#define STEP(init)                                                  \
    {                                                               \
        PlannerPosition q = (init);                                        \
        if (is_valid(q) && visited.find(q) == visited.end()) { \
            visited.insert(q);                                      \
            Q1.push_back(q);                                        \
        }                                                           \
    }

        STEP(move_forward(p));
        STEP(rotate(p));
        STEP(counter_rotate(p));

#undef STEP
    }

    ASSERT(false, "not found path");
    return 0;
}

PlannerSolver::PlannerSolver(uint32_t rows, uint32_t cols, std::vector<bool> map, std::vector<Position> robots_pos,
                             std::vector<int> robots_target, uint64_t random_seed) : rows(rows), cols(cols),
                                                                                     map(std::move(map)),
                                                                                     rnd(random_seed) {
    robots.resize(robots_pos.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        robots[r].start.x = robots_pos[r].x;
        robots[r].start.y = robots_pos[r].y;
        robots[r].start.pos = robots_pos[r].pos;
        robots[r].start.dir = robots_pos[r].dir;
        robots[r].target = robots_target[r];
    }
}


[[nodiscard]] SolutionInfo PlannerSolver::get_solution_info() const {
    SolutionInfo info;

    // TODO: тут на самом деле еще и то, что ребра тоже нужны в этой карте

    std::vector<std::vector<uint32_t>> map_cnt(PLANNER_DEPTH, std::vector<uint32_t>(map.size()));
    for (const auto &robot: robots) {
        PlannerPosition p = robot.start;
        for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
            p = simulate_action(p, robot.actions[d]);
            if (is_valid(p)) {

                if (!map[p.pos]) {
                    // препятствие на карте
                    info.collision_count++;
                }

                map_cnt[d][p.pos]++;
            }
        }
    }

    for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
        for (uint32_t pos = 0; pos < map.size(); pos++) {
            info.collision_count += map_cnt[d][pos] * (map_cnt[d][pos] - 1);
        }
    }

    return info;
}


void PlannerSolver::run(int time_limit) {
}
