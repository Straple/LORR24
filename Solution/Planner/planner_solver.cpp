#include "planner_solver.hpp"

#include "environment.hpp"

void PlannerSolver::init() {
    map_robots_cnt.resize(PLANNER_DEPTH, std::vector<uint32_t>(get_env().get_size()));
    map_edge_robots_cnt_ver.resize(PLANNER_DEPTH, std::vector<uint32_t>(get_env().get_size()));
    map_edge_robots_cnt_gor.resize(PLANNER_DEPTH, std::vector<uint32_t>(get_env().get_size()));

    pos_to_robot.resize(get_env().get_size(), -1);
    for (uint32_t r = 0; r < robots.size(); r++) {
        ASSERT(pos_to_robot[robots[r].start.pos] == -1, "pos_to_robot already init by other robot");
        pos_to_robot[robots[r].start.pos] = r;
    }

    // init robots pathes
    for (uint32_t r = 0; r < robots.size(); r++) {
        add_robot_path(r);
    }
}

PlannerSolver::PlannerSolver(std::vector<Position> robots_pos, std::vector<int> robots_target) {
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
    if (pos > to) {
        std::swap(pos, to);
    }
    if (to - pos == 1) {
        // gor
        cur_info.collision_count -= map_edge_robots_cnt_gor[d][pos] * (map_edge_robots_cnt_gor[d][pos] - 1);
        map_edge_robots_cnt_gor[d][pos] += val;
        cur_info.collision_count += map_edge_robots_cnt_gor[d][pos] * (map_edge_robots_cnt_gor[d][pos] - 1);
    } else {
        // ver
        ASSERT(to - pos == get_env().get_cols(), "invalid pos and to");

        cur_info.collision_count -= map_edge_robots_cnt_ver[d][pos] * (map_edge_robots_cnt_ver[d][pos] - 1);
        map_edge_robots_cnt_ver[d][pos] += val;
        cur_info.collision_count += map_edge_robots_cnt_ver[d][pos] * (map_edge_robots_cnt_ver[d][pos] - 1);
    }
}

void PlannerSolver::process_robot_path(uint32_t r, int sign) {
    auto robot = robots[r];
    Position p = robot.start;
    int t = 0;
    int best_dist = get_env().get_dist(p, robot.target);
    for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
        Position to = p.simulate_action(robot.actions[d]);
        if (to.is_valid()) {
            if (robot.actions[d] == Action::FW) {
                cur_info.count_forward += sign;
                change_map_edge_robots_cnt(t, p.pos, to.pos, sign);
            }

            change_map_robots_cnt(t, to.pos, sign);
            best_dist = min(best_dist, get_env().get_dist(to, robot.target));

            p = to;
            t++;
        }
    }
    while (t < PLANNER_DEPTH) {
        change_map_robots_cnt(t, p.pos, sign);
        t++;
    }
    cur_info.sum_dist_change += sign * (get_env().get_dist(robot.start, robot.target) - best_dist);
}

void PlannerSolver::add_robot_path(uint32_t r) {
    process_robot_path(r, +1);
}

void PlannerSolver::remove_robot_path(uint32_t r) {
    process_robot_path(r, -1);
}

SolutionInfo PlannerSolver::get_solution_info() const {
    return cur_info;
}

SolutionInfo PlannerSolver::get_trivial_solution_info() const {
    SolutionInfo info;

    // calc collision_count
    {
        std::vector<std::vector<uint32_t>> map_cnt(PLANNER_DEPTH, std::vector<uint32_t>(get_env().get_size()));
        std::vector<std::map<std::pair<uint32_t, uint32_t>, uint32_t>> edge_map_cnt(PLANNER_DEPTH);
        for (const auto &robot: robots) {
            Position p = robot.start;
            int t = 0;
            for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
                Position to = p.simulate_action(robot.actions[d]);
                if (to.is_valid()) {
                    if (robot.actions[d] == Action::FW) {
                        info.count_forward++;
                        int a = to.pos;
                        int b = p.pos;
                        if (a > b) {
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
            for (uint32_t pos = 0; pos < get_env().get_size(); pos++) {
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
    }

    // calc mean_dist_change
    /*{
        long long total_sum = 0;
        for (auto &robot: robots) {
            Position p = robot.start;
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

            total_sum += get_dist(robot.start, robot.target) -
                         best_dist;//static_cast<double>(get_dist(robot.start, robot.target) - best_dist) / PLANNER_DEPTH;
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

    return info;
}

double PlannerSolver::get_x(SolutionInfo info) const {
    double fw = info.count_forward;
    fw /= robots.size();
    fw /= PLANNER_DEPTH;

    double dist_change = info.sum_dist_change;
    dist_change /= robots.size();
    dist_change /= PLANNER_DEPTH;

    return 30 * dist_change  //
           - info.collision_count * 1e5 * info.collision_count //
           + fw;
}

bool PlannerSolver::compare(SolutionInfo old, SolutionInfo cur) {
    double oldx = get_x(old);
    double curx = get_x(cur);
    return oldx <= curx || rnd.get_d() < exp((curx - oldx) / temp);
    //return oldx <= curx || curx <= oldx * (1 + score_vector + rnd.get_d(0, 0.003));
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

bool PlannerSolver::try_move_over() {
    SolutionInfo old = get_solution_info();

    uint32_t r = rnd.get(0, static_cast<int>(robots.size()) - 1);

    // найдем рандомную цепочку перемещений
    Position v = robots[r].start;
    std::vector<int> dirs = {0, 1, 2, 3};

    std::set<int> visited;

    // [robot] = actions
    std::map<int, Actions> new_actions;
    while (true) {
        visited.insert(v.pos);

        int r = pos_to_robot[v.pos];
        if (r == -1) {
            break;
        }

        shuffle(dirs.begin(), dirs.end(), rnd.generator);

        bool find = false;
        for (int dir: dirs) {
            v.dir = dir;
            auto to = v.move_forward();
            if (to.is_valid() && !visited.count(to.pos)) {
                find = true;
                // вычислим необходимые действия для этого робота, чтобы попасть в to

                std::vector<Action> a;
                {
                    Position p = robots[r].start;
                    while (p.dir != dir) {
                        a.push_back(Action::CR);
                        p = p.rotate();
                    }
                    a.push_back(Action::FW);
                    p = p.move_forward();
                    ASSERT(p == to, "invalid move");
                }
                std::vector<Action> b;
                {
                    Position p = robots[r].start;
                    while (p.dir != dir) {
                        b.push_back(Action::CCR);
                        p = p.counter_rotate();
                    }
                    b.push_back(Action::FW);
                    p = p.move_forward();
                    ASSERT(p == to, "invalid move");
                }

                if (a.size() > b.size()) {
                    std::swap(a, b);
                }

                // a is best

                Actions new_action{Action::W};
                for (uint32_t k = 0; k < PLANNER_DEPTH && k < a.size(); k++) {
                    new_action[k] = a[k];
                }
                for (uint32_t k = a.size(); k < PLANNER_DEPTH; k++) {
                    new_action[k] = static_cast<Action>(rnd.get(0, 3));
                }
                new_actions[r] = new_action;

                v = to;
                break;
            }
        }

        if (!find) {
            return false;
        }
    }

    for (auto &[r, new_action]: new_actions) {
        remove_robot_path(r);
        std::swap(robots[r].actions, new_action);
        add_robot_path(r);
    }

    return consider(old, [&]() {
        for (auto &[r, new_action]: new_actions) {
            remove_robot_path(r);
            std::swap(robots[r].actions, new_action);
            add_robot_path(r);
        }
    });
}

void PlannerSolver::run(TimePoint end_time, uint64_t random_seed) {
    rnd = Randomizer(random_seed);

    temp = 1;
    for (int step = 0;; step++) {
        if (step % 10 == 0) {
            if (std::chrono::steady_clock::now() >= end_time) {
                break;
            }
        }
        //temp = (PLANNING_STEPS - step) * 0.1 / PLANNING_STEPS;
        temp *= 0.999;

        int k = rnd.get(0, 6);

        if (k == 0) {
            try_change_robot_action();
        } else if (k == 1) {
            try_change_robot_path();
        } else if (k == 2) {
            try_change_many_robots();
        } else {
            try_move_over();
        }

        /*scores.push_back(get_x(get_solution_info()));
        if (scores.size() > 100) {
            scores.erase(scores.begin());
        }
        double a = 0, b = 0;
        for (int i = 0; i < scores.size() / 2; i++) {
            a += scores[i];
        }
        for (int i = scores.size() / 2; i < scores.size(); i++) {
            b += scores[i];
        }
        score_vector = (b - a) / (std::abs(a) + std::abs(b) + 1);*/
    }
    /*auto end = std::chrono::steady_clock::now();
    std::cout << get_solution_info().collision_count << ' ' //
              << get_solution_info().sum_dist_change << ' ' //
              << get_solution_info().count_forward << ' ' //
              << std::chrono::duration_cast<milliseconds>(end - start).count() << "ms" << std::endl;
    ASSERT(get_solution_info().collision_count == 0, "invalid collision count");*/
}

std::pair<SolutionInfo, std::vector<Action>> PlannerSolver::get() const {
    std::vector<Action> actions(robots.size());

    // рассмотреть роботов
    for (uint32_t r = 0; r < robots.size(); r++) {
        actions[r] = Action::W;
        const Position p = robots[r].start;
        for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
            Position to = p.simulate_action(robots[r].actions[d]);
            if (to.is_valid()) {
                actions[r] = robots[r].actions[d];
                break;
            }
        }
    }

    return {get_solution_info(), actions};
}
