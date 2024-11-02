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
    return 0 <= p.x && p.x < rows && //
           0 <= p.y && p.y < cols && //
           map[p.pos];
}

int PlannerSolver::get_dist(PlannerPosition source, int target) const {
    return dist_dp[target][source.pos][source.dir];

    vector<PlannerPosition> Q0, Q1;
    Q0.push_back(source);

    std::set<PlannerPosition> visited;
    visited.insert(source);

    ASSERT(is_valid(source), "source is invalid");

    int d = 0;
    while (!Q0.empty() || !Q1.empty()) {
        if (Q0.empty()) {
            std::swap(Q0, Q1);
            d++;
        }

        ASSERT(!Q0.empty(), "Q0 is empty");
        PlannerPosition p = Q0.back();
        Q0.pop_back();

        ASSERT(is_valid(p), "p is invalid");

        if (p.pos == target) {
            //std::cout << "dist: " << dist_dp[p.pos][source.pos][source.dir] << ' ' << d << std::endl;
            ASSERT(dist_dp[p.pos][source.pos][source.dir] == d, "invalid dist_dp");
            return d;
        }

#define STEP(init)                                                  \
    {                                                               \
        PlannerPosition to = (init);                                \
        if (is_valid(to) && visited.find(to) == visited.end()) {    \
            visited.insert(to);                                     \
            Q1.push_back(to);                                       \
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

void PlannerSolver::build_dist(int target) {
    PlannerPosition planner_source = {target / static_cast<int>(cols), target % static_cast<int>(cols), target, 0};
    if (!is_valid(planner_source)) {
        return;
    }

    dist_dp[target].assign(map.size(), std::vector<int>(4));

    vector<PlannerPosition> Q0, Q1;
    std::set<PlannerPosition> visited;
    {

        for (int dir = 0; dir < 4; dir++) {
            planner_source.dir = dir;
            Q0.push_back(planner_source);
            visited.insert(planner_source);
        }
    }

    int d = 0;
    while (!Q0.empty() || !Q1.empty()) {
        if (Q0.empty()) {
            std::swap(Q0, Q1);
            d++;
        }

        ASSERT(!Q0.empty(), "Q0 is empty");
        PlannerPosition p = Q0.back();
        Q0.pop_back();

        dist_dp[target][p.pos][(p.dir + 2) % 4] = d;

        ASSERT(is_valid(p), "p is invalid");

#define STEP(init)                                                  \
    {                                                               \
        PlannerPosition to = (init);                                \
        if (is_valid(to) && visited.find(to) == visited.end()) {    \
            visited.insert(to);                                     \
            Q1.push_back(to);                                       \
        }                                                           \
    }

        STEP(move_forward(p));
        STEP(rotate(p));
        STEP(counter_rotate(p));

#undef STEP
    }
}

void PlannerSolver::build_dist() {
    dist_dp.resize(map.size());
    for (int target = 0; target < map.size(); target++) {
        build_dist(target);
    }
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

    build_dist();
}

[[nodiscard]] SolutionInfo PlannerSolver::get_solution_info() const {
    SolutionInfo info;

    // TODO: тут на самом деле еще и то, что ребра тоже нужны в этой карте
    // calc collision_count
    {
        std::vector<std::vector<uint32_t>> map_cnt(PLANNER_DEPTH, std::vector<uint32_t>(map.size()));
        for (const auto &robot: robots) {
            PlannerPosition p = robot.start;
            for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
                PlannerPosition to = simulate_action(p, robot.actions[d]);
                if (is_valid(to)) {
                    map_cnt[d][to.pos]++;
                    p = to;
                }
            }
        }

        for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
            for (uint32_t pos = 0; pos < map.size(); pos++) {
                info.collision_count += map_cnt[d][pos] * (map_cnt[d][pos] - 1);
            }
        }
    }

    // calc mean_dist_change
    {
        double total_sum = 0;
        int total_cnt = 0;
        for (const auto &robot: robots) {
            if (robot.target == -1) {
                continue;
            }
            PlannerPosition p = robot.start;
            int init_dist = get_dist(p, robot.target);
            int cnt_actions = 0;
            int sum_change_dist = 0;

            for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
                PlannerPosition to = simulate_action(p, robot.actions[d]);
                if (is_valid(to)) {
                    sum_change_dist += (init_dist - get_dist(to, robot.target));
                    cnt_actions++;
                    p = to;
                }
            }

            if (cnt_actions != 0) {
                total_sum += static_cast<double>(sum_change_dist) / cnt_actions;
                total_cnt++;
            }
        }

        info.mean_dist_change = total_sum / total_cnt;
    }

    return info;
}

bool PlannerSolver::compare(SolutionInfo old, SolutionInfo cur) const {
    return old.mean_dist_change - old.collision_count * 10 < cur.mean_dist_change - cur.collision_count * 10;
}

bool PlannerSolver::try_change_robot_action() {

    SolutionInfo old = get_solution_info();

    // берем робота
    uint32_t r = rnd.get(0, static_cast<int>(robots.size()) - 1);

    // берем операцию у него
    uint32_t d = rnd.get(0, PLANNER_DEPTH - 1);

    Action old_action = robots[r].actions[d];

    robots[r].actions[d] = static_cast<Action>(rnd.get(0, 3));

    return consider(old, [&]() {
        robots[r].actions[d] = old_action;
    });
}

void PlannerSolver::run(int time_limit) {
    for (int step = 0; step < 1000; step++) {
        try_change_robot_action();
    }
    std::cout << get_solution_info().collision_count << ' ' << get_solution_info().mean_dist_change << std::endl;
    ASSERT(get_solution_info().collision_count == 0, "invalid collision count");
}

std::pair<SolutionInfo, std::vector<Action>> PlannerSolver::get() const {
    std::vector<Action> actions(robots.size());

    // рассмотреть роботов
    for (uint32_t r = 0; r < robots.size(); r++) {
        actions[r] = Action::W;
        for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
            PlannerPosition p = robots[r].start;
            p = simulate_action(p, robots[r].actions[d]);
            if (is_valid(p)) {
                actions[r] = robots[r].actions[d];
                break;
            }
        }
    }

    return {get_solution_info(), actions};
}
