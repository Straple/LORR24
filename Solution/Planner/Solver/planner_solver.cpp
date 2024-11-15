#include "planner_solver.hpp"

#include "../../Objects/environment.hpp"
#include "global_dp.hpp"

bool PlannerSolver::is_valid(const Position &p) const {
    ASSERT(!robots.empty(), "robots is empty");
    return p.is_valid() && get_env().get_major(p.pos) == get_env().get_major(robots[0].start.pos);
}

void PlannerSolver::init() {
    // init robots pathes
    for (int r = 0; r < robots.size(); r++) {
        robot_to_idx[get_global_dp().get_robot(robots[r].start.pos)] = r;
    }
    for (uint32_t r = 0; r < robots.size(); r++) {
        add_robot_path(r);
    }
}

void PlannerSolver::update_answer() {
    if (get_x(solution_info) > get_x(answer_info)) {
        answer_info = solution_info;
        for (uint32_t r = 0; r < robots.size(); r++) {
            answer_actions[r] = Action::W;
            const Position p = robots[r].start;
            for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
                Position to = p.simulate_action(robots[r].actions[d]);
                if (is_valid(to)) {
                    answer_actions[r] = robots[r].actions[d];
                    break;
                }
            }
        }
    }
}

PlannerSolver::PlannerSolver(std::vector<Position> robots_pos, std::vector<int> robots_target) {
    ASSERT(robots_pos.size() == robots_target.size(), "invalid sizes");
    robots.resize(robots_pos.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        robots[r].start.x = robots_pos[r].x;
        robots[r].start.y = robots_pos[r].y;
        robots[r].start.pos = robots_pos[r].pos;
        robots[r].start.dir = robots_pos[r].dir;
        robots[r].target = robots_target[r];
    }

    init();

    answer_actions.resize(robots.size(), Action::W);
    answer_info = solution_info;
}

void PlannerSolver::process_robot_path(uint32_t r, int sign) {
    ASSERT(r < robots.size(), "invalid r");
    auto robot = robots[r];
    Position p = robot.start;
    ASSERT(is_valid(p), "invalid p");
    int t = 0;
    int best_dist = get_env().get_dist(p, robot.target);
    std::array<int32_t, PLANNER_DEPTH + 1> dists{};
    dists[0] = get_env().get_dist(p, robot.target);
    for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
        Position to = p.simulate_action(robot.actions[d]);
        if (is_valid(to)) {
            if (robot.actions[d] == Action::FW) {
                solution_info.count_forward[t] += sign;
                get_global_dp().change_map_edge_robots_cnt(t, p.pos, to.pos, sign, solution_info);
            }

            dists[t + 1] = get_env().get_dist(to, robot.target);
            //solution_info.sum_dist_change[t] += sign * (get_env().get_dist(p, robot.target) - get_env().get_dist(to, robot.target));

            get_global_dp().change_map_robots_cnt(t, to.pos, sign, solution_info);
            best_dist = min((int64_t)best_dist, get_env().get_dist(to, robot.target));

            p = to;
            t++;
        }
    }
    while (t < PLANNER_DEPTH) {
        dists[t + 1] = get_env().get_dist(p, robot.target);
        //solution_info.sum_dist_change[t] += sign * (get_env().get_dist(p, robot.target) - get_env().get_dist(p, robot.target));
        get_global_dp().change_map_robots_cnt(t, p.pos, sign, solution_info);
        t++;
    }
    //solution_info.sum_dist_change += sign * (get_env().get_dist(robot.start, robot.target) - best_dist);

    for (uint32_t t = 0; t < PLANNER_DEPTH; t++) {
        solution_info.sum_dist_change[t] += sign * (dists[t] - dists[t + 1]);
        //for(int b = 0; b < t; b++){
        //    solution_info.sum_dist_change[t] += sign * (dists[b] - dists[t]);
        //}
    }
}

void PlannerSolver::add_robot_path(uint32_t r) {
    process_robot_path(r, +1);
}

void PlannerSolver::remove_robot_path(uint32_t r) {
    process_robot_path(r, -1);
}

SolutionInfo PlannerSolver::get_solution_info() const {
    return solution_info;
}

SolutionInfo PlannerSolver::get_trivial_solution_info() const {
    //return solution_info;
    SolutionInfo info;

    // calc collision_count
    /*{
        std::vector<std::vector<uint32_t>> map_cnt(PLANNER_DEPTH, std::vector<uint32_t>(get_env().get_size()));
        std::vector<std::map<std::pair<uint32_t, uint32_t>, uint32_t>> edge_map_cnt(PLANNER_DEPTH);
        for (const auto &robot: robots) {
            Position p = robot.start;
            int t = 0;
            for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
                Position to = p.simulate_action(robot.actions[d]);
                if (is_valid(to)) {
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
                //if (map_robots_cnt[d][pos] != map_cnt[d][pos]) {
                //    std::cout << "diff: " << d << ' ' << pos << ' ' << map_robots_cnt[d][pos] << " != " <<
                //              map_cnt[d][pos] << std::endl;
                //}
                info.collision_count += map_cnt[d][pos] * (map_cnt[d][pos] - 1);
            }

            for (auto [edge, cnt]: edge_map_cnt[d]) {
                info.collision_count += cnt * (cnt - 1);
            }
        }

        //ASSERT(map_robots_cnt == map_cnt, "invalid map_cnt");

        ASSERT(info.collision_count == solution_info.collision_count,
               "invalid collision count: " + std::to_string(info.collision_count) +
                       " != " + std::to_string(solution_info.collision_count));
    }*/

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
    /*info.sum_dist_change = solution_info.sum_dist_change;

    if (info != solution_info) {
        std::cout << info << '\n'
                  << solution_info << std::endl;
    }
    ASSERT(info == solution_info, "invalid solution info");*/

    return info;
}

double PlannerSolver::get_x(const SolutionInfo &info) const {

    constexpr double r = 0.95;

    const double norm = 1 / static_cast<double>(robots.size());

    double res = 0;
    double m = 1;
    for (uint32_t k = 0; k < PLANNER_DEPTH; k++) {
        res -= info.collision_count[k] * static_cast<double>(robots.size()) * 0.5 * m;
        res += info.sum_dist_change[k] * m;
        //res += info.count_forward[k] / norm * m;

        m *= r;
    }
    return res;

    /*double fw = info.count_forward;
    fw /= robots.size();
    fw /= PLANNER_DEPTH;

    double dist_change = info.sum_dist_change;
    dist_change /= robots.size();
    dist_change /= PLANNER_DEPTH;

    return 30 * dist_change                                   //
           - info.collision_count * 1e5 * info.collision_count//
           + fw;*/
}

bool PlannerSolver::compare(SolutionInfo old, SolutionInfo cur, Randomizer &rnd) {
    update_answer();

    double oldx = get_x(old);
    double curx = get_x(cur);
    return oldx <= curx || rnd.get_d() < exp((curx - oldx) / temp);
    //return oldx <= curx || curx <= oldx * (1 + score_vector + rnd.get_d(0, 0.003));
}

bool PlannerSolver::try_change_robot_action(Randomizer &rnd) {
    SolutionInfo old = get_solution_info();

    // берем робота
    uint32_t r = rnd.get(0, static_cast<int>(robots.size()) - 1);

    // берем операцию у него
    uint32_t d = rnd.get(0, PLANNER_DEPTH - 1);

    Action old_action = robots[r].actions[d];

    remove_robot_path(r);
    robots[r].actions[d] = static_cast<Action>(rnd.get(0, 3));
    add_robot_path(r);

    return consider(old, rnd, [&]() {
        remove_robot_path(r);
        robots[r].actions[d] = old_action;
        add_robot_path(r);
    });
}

bool PlannerSolver::try_change_robot_path(Randomizer &rnd) {
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

    return consider(old, rnd, [&]() {
        remove_robot_path(r);
        robots[r].actions = old_actions;
        add_robot_path(r);
    });
}

bool PlannerSolver::try_change_many_robots(Randomizer &rnd) {
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

    return consider(old, rnd, [&]() {
        remove_robot_path(r1);
        remove_robot_path(r2);
        robots[r1].actions = old_actions1;
        robots[r2].actions = old_actions2;
        add_robot_path(r1);
        add_robot_path(r2);
    });
}

bool PlannerSolver::try_move_over(Randomizer &rnd) {
    SolutionInfo old = get_solution_info();

    uint32_t r = rnd.get(0, static_cast<int>(robots.size()) - 1);

    // найдем рандомную цепочку перемещений
    Position v = robots[r].start;

    std::set<int> visited;

    //static std::vector<int> dirs = {0, 1, 2, 3};

    // [robot] = actions
    std::map<int, Actions> new_actions;
    while (true) {
        visited.insert(v.pos);

        int r = get_global_dp().get_robot(v.pos);
        if (r == -1) {
            break;
        }
        ASSERT(robot_to_idx.count(r), "no contains r");
        r = robot_to_idx[r];
        ASSERT(0 <= r && r < robots.size(), "invalid r");

        //uint64_t x = rnd.get();
        v.dir = robots[r].start.dir;
        Actions new_actiona{Action::W}, new_actionb{Action::W};

        std::vector<std::pair<Actions, Position>> data;
        Position a = v;
        Position b = v;
        for (int k = 0; k < 4; k++) {
            auto toa = a.move_forward();
            if (is_valid(toa) && !visited.count(toa.pos)) {
                if (k < PLANNER_DEPTH) {
                    new_actiona[k] = Action::FW;
                }
                data.emplace_back(new_actiona, toa);
            }

            auto tob = b.move_forward();
            if (is_valid(tob) && !visited.count(tob.pos)) {
                if (k < PLANNER_DEPTH) {
                    new_actionb[k] = Action::FW;
                }
                data.emplace_back(new_actionb, tob);
            }

            if (k < PLANNER_DEPTH) {
                new_actiona[k] = Action::CR;
            }
            a = a.rotate();

            if (k < PLANNER_DEPTH) {
                new_actionb[k] = Action::CCR;
            }
            b = b.counter_rotate();
        }

        if (data.empty()) {
            break;
        }

        if (rnd.get() & 1) {
            auto [new_action, to] = data[0];
            new_actions[r] = new_action;
            v = to;
        } else {
            auto [new_action, to] = rnd.get(data);
            new_actions[r] = new_action;
            v = to;
        }
    }

    for (auto &[r, new_action]: new_actions) {
        remove_robot_path(r);
        std::swap(robots[r].actions, new_action);
        add_robot_path(r);
    }

    return consider(old, rnd, [&]() {
        for (auto &[r, new_action]: new_actions) {
            remove_robot_path(r);
            std::swap(robots[r].actions, new_action);
            add_robot_path(r);
        }
    });
}

void PlannerSolver::run(TimePoint end_time, uint64_t random_seed) {
    Randomizer rnd(random_seed);

    temp = 0.3;
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
            try_change_robot_action(rnd);
        } else if (k == 1) {
            try_change_robot_path(rnd);
        } else if (k == 2) {
            try_change_many_robots(rnd);
        } else {
            try_move_over(rnd);
        }
    }
}

std::pair<SolutionInfo, std::vector<Action>> PlannerSolver::get() const {
    return {answer_info, answer_actions};

    /*std::vector<Action> actions(robots.size());

    for (uint32_t r = 0; r < robots.size(); r++) {
        actions[r] = Action::W;
        const Position p = robots[r].start;
        for (uint32_t d = 0; d < PLANNER_DEPTH; d++) {
            Position to = p.simulate_action(robots[r].actions[d]);
            if (is_valid(to)) {
                actions[r] = robots[r].actions[d];
                break;
            }
        }
    }

    return {get_solution_info(), actions};*/
}
