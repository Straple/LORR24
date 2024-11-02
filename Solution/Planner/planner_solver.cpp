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
    if (target == -1) {
        return 0;
    }
    ASSERT(0 <= target && target < map.size(), "invalid target: " + std::to_string(target));
    ASSERT(0 <= source.pos && source.pos < map.size(), "invalid source: " + std::to_string(source.pos));
    ASSERT(0 <= source.dir && source.dir < 4, "invalid dir: " + std::to_string(source.dir));

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
    auto start = std::chrono::steady_clock::now();
    dist_dp.resize(map.size());
    for (int target = 0; target < map.size(); target++) {
        build_dist(target);
    }

    auto end = std::chrono::steady_clock::now();
    std::cout << "build dist time: " << std::chrono::duration_cast<milliseconds>(end - start).count() << std::endl;
}

void PlannerSolver::init() {
    map_robots_cnt.resize(PLANNER_DEPTH, std::vector<uint32_t>(map.size()));
    map_edge_robots_cnt.resize(PLANNER_DEPTH, std::vector<uint32_t>(4 * map.size()));

    // build edge_to_idx
    {
        for (int x = 0; x < rows; x++) {
            for (int y = 0; y < cols; y++) {
                int pos = x * cols + y;
                if (x + 1 < rows) {
                    int to = pos + cols;
                    edge_to_idx[{pos, to}] = -1;
                }
                if (y + 1 < cols) {
                    int to = pos + 1;
                    edge_to_idx[{pos, to}] = -1;
                }
            }
        }

        uint32_t idx = 0;
        for (auto &[edge, val]: edge_to_idx) {
            val = idx;
            idx++;
        }

        auto tmp = std::move(edge_to_idx);
        for (auto [edge, val]: tmp) {
            edge_to_idx[edge] = val;
            edge_to_idx[{edge.second, edge.first}] = val;
        }
    }

    // init robots pathes
    for (uint32_t r = 0; r < robots.size(); r++) {
        add_robot_path(r);
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

    init();
}

void PlannerSolver::change_map_robots_cnt(int d, int pos, int val) {
    cur_info.collision_count -= map_robots_cnt[d][pos] * (map_robots_cnt[d][pos] - 1);
    map_robots_cnt[d][pos] += val;
    cur_info.collision_count += map_robots_cnt[d][pos] * (map_robots_cnt[d][pos] - 1);
}

void PlannerSolver::change_map_edge_robots_cnt(int d, int pos, int to, int val) {
    ASSERT(edge_to_idx.count({pos, to}) == 1, "no contains");
    uint32_t idx = edge_to_idx[{pos, to}];
    ASSERT(idx < map_edge_robots_cnt[d].size(), "invalid idx: " + std::to_string(idx));

    cur_info.collision_count -= map_edge_robots_cnt[d][idx] * (map_edge_robots_cnt[d][idx] - 1);
    map_edge_robots_cnt[d][idx] += val;
    cur_info.collision_count += map_edge_robots_cnt[d][idx] * (map_edge_robots_cnt[d][idx] - 1);
}

void PlannerSolver::add_robot_path(uint32_t r) {
    auto robot = robots[r];
    PlannerPosition p = robot.start;
    int t = 0;
    int best_dist = get_dist(p, robot.target);
    for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
        PlannerPosition to = simulate_action(p, robot.actions[d]);
        if (is_valid(to)) {
            if (robot.actions[d] == Action::FW) {
                cur_info.count_forward++;
                change_map_edge_robots_cnt(t, p.pos, to.pos, +1);
            }

            change_map_robots_cnt(t, to.pos, +1);
            best_dist = min(best_dist, get_dist(to, robot.target));

            p = to;
            t++;
        }
    }
    while (t < PLANNER_DEPTH) {
        change_map_robots_cnt(t, p.pos, +1);
        t++;
    }
    cur_info.sum_dist_change += get_dist(robot.start, robot.target) - best_dist;
}

void PlannerSolver::remove_robot_path(uint32_t r) {
    auto robot = robots[r];
    PlannerPosition p = robot.start;
    int t = 0;
    int best_dist = get_dist(p, robot.target);
    for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
        PlannerPosition to = simulate_action(p, robot.actions[d]);
        if (is_valid(to)) {
            if (robot.actions[d] == Action::FW) {
                cur_info.count_forward--;
                change_map_edge_robots_cnt(t, p.pos, to.pos, -1);
            }
            change_map_robots_cnt(t, to.pos, -1);
            best_dist = min(best_dist, get_dist(to, robot.target));

            p = to;
            t++;
        }
    }
    while (t < PLANNER_DEPTH) {
        change_map_robots_cnt(t, p.pos, -1);
        t++;
    }
    cur_info.sum_dist_change -= get_dist(robot.start, robot.target) - best_dist;
}

[[nodiscard]] SolutionInfo PlannerSolver::get_solution_info() const {
    SolutionInfo info;

    // calc collision_count
    /*{
        std::vector<std::vector<uint32_t>> map_cnt(PLANNER_DEPTH, std::vector<uint32_t>(map.size()));
        std::vector<std::map<std::pair<uint32_t, uint32_t>, uint32_t>> edge_map_cnt(PLANNER_DEPTH);
        for (const auto &robot: robots) {
            PlannerPosition p = robot.start;
            int t = 0;
            for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
                PlannerPosition to = simulate_action(p, robot.actions[d]);
                if (is_valid(to)) {
                    if (robot.actions[d] == Action::FW) {
                        int a = to.pos;
                        int b = p.pos;
                        if(a > b){
                            std::swap(a, b);
                        }
                        edge_map_cnt[t][{a, b}]++;
                    }
                    map_cnt[t][to.pos]++;
                    p = to;
                    t++;
                }
            }
            while (t < PLANNER_DEPTH) {
                map_cnt[t][p.pos]++;
                t++;
            }
        }

        for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
            for (uint32_t pos = 0; pos < map.size(); pos++) {
                if (map_robots_cnt[d][pos] != map_cnt[d][pos]) {
                    std::cout << "diff: " << d << ' ' << pos << ' ' << map_robots_cnt[d][pos] << " != " <<
                              map_cnt[d][pos] << std::endl;
                }
                info.collision_count += map_cnt[d][pos] * (map_cnt[d][pos] - 1);
            }

            for (auto [edge, cnt]: edge_map_cnt[d]) {
                info.collision_count += cnt * (cnt - 1);
            }
        }

        ASSERT(map_robots_cnt == map_cnt, "invalid map_cnt");

        ASSERT(info.collision_count == cur_info.collision_count,
               "invalid collision count: " + std::to_string(info.collision_count) +
               " != " + std::to_string(cur_info.collision_count));
    }*/
    info.collision_count = cur_info.collision_count;

    // calc mean_dist_change
    /*{
        long long total_sum = 0;
        for (auto &robot: robots) {
            PlannerPosition p = robot.start;
            int init_dist = get_dist(p, robot.target);
            int sum_change_dist = 0;

            int best_dist = get_dist(p, robot.target);

            int t = 0;
            for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
                PlannerPosition to = simulate_action(p, robot.actions[d]);
                if (is_valid(to)) {
                    info.count_forward += robot.actions[d] == Action::FW;
                    best_dist = min(best_dist, get_dist(to, robot.target));
                    sum_change_dist += (get_dist(p, robot.target) - get_dist(to, robot.target));
                    p = to;
                    t++;
                }
            }

            while (t < PLANNER_DEPTH) {
                sum_change_dist += 0;//(init_dist - get_dist(p, robot.target));
                t++;
            }

            total_sum += get_dist(robot.start, robot.target) - best_dist;//static_cast<double>(get_dist(robot.start, robot.target) - best_dist) / PLANNER_DEPTH;
        }

        ASSERT(info.count_forward == cur_info.count_forward,
               "invalid forward: " + std::to_string(info.count_forward) + " != " +
               std::to_string(cur_info.count_forward));

        info.sum_dist_change = total_sum;

        ASSERT(info.sum_dist_change == cur_info.sum_dist_change, "invalid sum dist change");
        //info.mean_dist_change = total_sum / total_cnt;
        //info.mean_dist_change /= PLANNER_DEPTH;
    }*/
    info.sum_dist_change = cur_info.sum_dist_change;

    info.count_forward = cur_info.count_forward;

    return info;
}

double PlannerSolver::get_x(SolutionInfo info) {
    double fw = info.count_forward;
    fw /= robots.size();
    fw /= PLANNER_DEPTH;

    double dist_change = info.sum_dist_change;
    dist_change /= robots.size();
    dist_change /= PLANNER_DEPTH;

    return dist_change  //
           - info.collision_count * 10 //
           + 1e-1 * fw;
}

bool PlannerSolver::compare(SolutionInfo old, SolutionInfo cur) {
    double oldx = get_x(old);
    double curx = get_x(cur);
    return oldx <= curx || rnd.get_d() < exp((curx - oldx) / temp);
}

bool PlannerSolver::try_change_robot_action() {
    SolutionInfo old = get_solution_info();

    // берем робота
    uint32_t r = rnd.get(0, static_cast<int>(robots.size()) - 1);

    // берем операцию у него
    uint32_t d = rnd.get(0, PLANNER_DEPTH - 1);

    Action old_action = robots[r].actions[d];

    remove_robot_path(r);
    robots[r].actions[d] = static_cast<Action>(rnd.get(0, 3));
    add_robot_path(r);

    return consider(old, [&]() {
        remove_robot_path(r);
        robots[r].actions[d] = old_action;
        add_robot_path(r);
    });
}

bool PlannerSolver::try_change_robot_path() {
    SolutionInfo old = get_solution_info();

    // берем робота
    uint32_t r = rnd.get(0, static_cast<int>(robots.size()) - 1);

    std::array<Action, PLANNER_DEPTH> new_actions{};
    {
        uint64_t x = rnd.get();
        for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
            new_actions[d] = static_cast<Action>(x % 4);
            x /= 4;
        }
    }

    std::array<Action, PLANNER_DEPTH> old_actions = robots[r].actions;

    remove_robot_path(r);
    robots[r].actions = new_actions;
    add_robot_path(r);

    return consider(old, [&]() {
        remove_robot_path(r);
        robots[r].actions = old_actions;
        add_robot_path(r);
    });
}

bool PlannerSolver::try_change_many_robots() {
    SolutionInfo old = get_solution_info();

    uint32_t r1 = rnd.get(0, static_cast<int>(robots.size()) - 1);
    uint32_t r2 = rnd.get(0, static_cast<int>(robots.size()) - 1);

    if (r1 == r2) {
        return false;
    }

    std::array<Action, PLANNER_DEPTH> new_actions1{};
    {
        uint64_t x = rnd.get();
        for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
            new_actions1[d] = static_cast<Action>(x % 4);
            x /= 4;
        }
    }

    std::array<Action, PLANNER_DEPTH> new_actions2{};
    {
        uint64_t x = rnd.get();
        for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
            new_actions2[d] = static_cast<Action>(x % 4);
            x /= 4;
        }
    }

    std::array<Action, PLANNER_DEPTH> old_actions1 = robots[r1].actions;
    std::array<Action, PLANNER_DEPTH> old_actions2 = robots[r2].actions;

    remove_robot_path(r1);
    remove_robot_path(r2);
    robots[r1].actions = new_actions1;
    robots[r2].actions = new_actions2;
    add_robot_path(r1);
    add_robot_path(r2);

    return consider(old, [&]() {
        remove_robot_path(r1);
        remove_robot_path(r2);
        robots[r1].actions = old_actions1;
        robots[r2].actions = old_actions2;
        add_robot_path(r1);
        add_robot_path(r2);
    });
}

void PlannerSolver::run(int time_limit) {
    auto start = std::chrono::steady_clock::now();
    temp = 1;
    for (int step = 0; step < PLANNING_STEPS; step++) {
        //temp = (PLANNING_STEPS - step) * 1.0 / PLANNING_STEPS;
        temp *= 0.9999;

        int k = rnd.get(0, 2);

        if (k == 0) {
            try_change_robot_action();
        } else if (k == 1) {
            try_change_robot_path();
        } else if (k == 2) {
            try_change_many_robots();
        } else {
            ASSERT(false, "invalid k: " + std::to_string(k));
        }
    }
    auto end = std::chrono::steady_clock::now();
    std::cout << get_solution_info().collision_count << ' ' //
              << get_solution_info().sum_dist_change << ' ' //
              << get_solution_info().count_forward << ' ' //
              << std::chrono::duration_cast<milliseconds>(end - start).count() << "ms" << std::endl;
    ASSERT(get_solution_info().collision_count == 0, "invalid collision count");
}

std::pair<SolutionInfo, std::vector<Action>> PlannerSolver::get() const {
    std::vector<Action> actions(robots.size());

    // рассмотреть роботов
    for (uint32_t r = 0; r < robots.size(); r++) {
        actions[r] = Action::W;
        PlannerPosition p = robots[r].start;
        for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
            PlannerPosition to = simulate_action(p, robots[r].actions[d]);
            if (is_valid(to)) {
                actions[r] = robots[r].actions[d];
                break;
            }
        }
    }

    return {get_solution_info(), actions};
}
