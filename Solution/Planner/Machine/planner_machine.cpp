#include "planner_machine.hpp"

#include "../../Objects/environment.hpp"

bool PlannerMachine::is_allowed(uint32_t k, Position p, Action action) const {
    auto to = p.simulate_action(action);
    if (!to.is_valid()) {
        return false;
    }
    if (map[k][to.pos] != -1) {
        return false;
    }
    if (action == Action::FW) {
        int pos = std::min(p.pos, to.pos);
        if (std::abs(to.pos - p.pos) == 1) {
            // gor
            if (map_gor[k][pos] != -1) {
                return false;
            }
        } else {
            // ver
            if (map_ver[k][pos] != -1) {
                return false;
            }
        }
    }
    return true;
}

std::vector<uint32_t> PlannerMachine::get_collisions(uint32_t k, Position p, Action action) const {
    auto to = p.simulate_action(action);
    ASSERT(to.is_valid(), "no valid");
    std::vector<uint32_t> ans;
    if (map[k][to.pos] != -1) {
        ans.push_back(map[k][to.pos]);
    }
    if (action == Action::FW) {
        int pos = std::min(p.pos, to.pos);
        if (std::abs(to.pos - p.pos) == 1) {
            // gor
            if (map_gor[k][pos] != -1) {
                ans.push_back(map_gor[k][pos]);
            }
        } else {
            // ver
            if (map_ver[k][pos] != -1) {
                ans.push_back(map_ver[k][pos]);
            }
        }
    }
    return ans;
}

void PlannerMachine::update_targets() {
    for (auto &robot: robots) {
        robot.target = -1;
    }

    for (auto &task: get_env().get_shared_env().task_pool) {
        if (task.agent_assigned != -1) {
            ASSERT(0 <= task.agent_assigned && task.agent_assigned < robots.size(), "invalid task agent assigned");
            robots[task.agent_assigned].target = task.get_next_loc();
            ASSERT(0 <= robots[task.agent_assigned].target && robots[task.agent_assigned].target < get_env().get_size(),
                   "invalid target");
        }
    }
}

void PlannerMachine::simulate_world() {
    for (auto &robot: robots) {
        robot.start = robot.start.simulate_action(robot.actions[0]);
        for (uint32_t k = 1; k < PLANNER_DEPTH; k++) {
            robot.actions[k - 1] = robot.actions[k];
        }
        robot.actions[PLANNER_DEPTH - 1] = Action::W;
    }
    for (uint32_t k = 1; k < PLANNER_DEPTH; k++) {
        map[k - 1] = map[k];
        map_gor[k - 1] = map_gor[k];
        map_ver[k - 1] = map_ver[k];
    }
    map_gor[PLANNER_DEPTH - 1].assign(get_env().get_size(), -1);
    map_ver[PLANNER_DEPTH - 1].assign(get_env().get_size(), -1);
}

std::optional<PlannerMachine::Actions> PlannerMachine::get_path(uint32_t r) const {
    struct State {
        double score = 0;
        int k = 0;
        Position p;
        Actions path;
    };

    // время и позиция
    std::set<std::pair<uint32_t, Position>> visited;

    struct comparator {
        bool operator()(const State &lhs, const State &rhs) const {
            return lhs.score < rhs.score;
        }
    };

    //constexpr double COLLISION_POWER = 20;

    std::multiset<State, comparator> S;
    S.insert(State{0, 0, robots[r].start, get_wactions()});

    uint32_t cnt = 0;
    while (!S.empty()) {
        cnt++;
        if(cnt > 100'000){
            break;
        }
        auto [score, k, p, path] = *S.begin();
        S.erase(S.begin());

        ASSERT(p.is_valid(), "p is invalid");

        if (visited.count({k, p})) {
            continue;
        }
        visited.insert({k, p});

        if (p.pos == robots[r].target && k >= PLANNER_DEPTH) {
            return path;
        }

        auto step = [&](Action action) {
            Position to = p.simulate_action(action);
            if (is_allowed(std::min(static_cast<uint32_t>(k), PLANNER_DEPTH - 1), p, action)) {
                if (k < PLANNER_DEPTH) {
                    path[k] = action;
                }
                if (!visited.count({k + 1, to})) {
                    S.insert({score + 1 + (action == Action::W) * 10, k + 1, to, path});
                }
            }
        };

        step(Action::FW);
        step(Action::CR);
        step(Action::CCR);
        step(Action::W);
    }

    return std::nullopt;
}

std::pair<std::vector<uint32_t>, PlannerMachine::Actions>
PlannerMachine::get_collision_path(uint32_t r, double collision_power) const {
    struct State {
        double score = 0;
        int k = 0;
        Position p;
        Actions path;
        std::vector<uint32_t> rids;
    };

    // время и позиция
    std::set<std::pair<uint32_t, Position>> visited;

    struct comparator {
        bool operator()(const State &lhs, const State &rhs) const {
            return lhs.score < rhs.score;
        }
    };

    std::multiset<State, comparator> S;
    S.insert(State{1.0 * get_env().get_dist(robots[r].start, robots[r].target), 0, robots[r].start, get_wactions()});

    while (!S.empty()) {
        auto [score, k, p, path, rids] = *S.begin();
        S.erase(S.begin());

        ASSERT(p.is_valid(), "p is invalid");

        if (visited.count({k, p})) {
            continue;
        }
        visited.insert({k, p});

        if (k == PLANNER_DEPTH) {
            return {rids, path};
        }

        auto step = [&](Action action) {
            Position to = p.simulate_action(action);
            if (!to.is_valid()) {
                return;
            }
            auto new_rids = rids;
            {
                auto vec = get_collisions(k, p, action);
                for (auto &id: vec) {
                    new_rids.push_back(id);
                }
                sort(new_rids.begin(), new_rids.end());
                new_rids.erase(unique(new_rids.begin(), new_rids.end()), new_rids.end());
            }
            path[k] = action;
            double new_score = score - 1;
            new_score -= rids.size() * collision_power;
            new_score += new_rids.size() * collision_power;
            new_score -= get_env().get_dist(p, robots[r].target);
            new_score += get_env().get_dist(to, robots[r].target);

            S.insert({new_score, k + 1, to, path, new_rids});
        };

        step(Action::FW);
        step(Action::CR);
        step(Action::CCR);
        step(Action::W);
    }

    FAILED_ASSERT("unable to find path");
    return {};
}

double PlannerMachine::get_dist(uint32_t k, Position source, int target, double collision_power) const {
    FAILED_ASSERT("outdated");
    ASSERT(target != -1, "pizdec");
    //return get_env().get_dist(source, target);

    struct State {
        double score = 0;
        uint32_t k = 0;
        Position p;
    };

    // время и позиция
    std::set<std::pair<uint32_t, Position>> visited;

    struct comparator {
        bool operator()(const State &lhs, const State &rhs) const {
            return lhs.score < rhs.score;
        }
    };

    std::multiset<State, comparator> S;
    S.insert(State{1.0 * get_env().get_dist(source, target), k, source});

    while (!S.empty()) {
        auto [score, k, p] = *S.begin();
        S.erase(S.begin());

        //std::cout << S.size() << ' ' << score << ' ' << k << ' ' << p.pos << ' ' << target << std::endl;

        ASSERT(p.is_valid(), "p is invalid");

        if (visited.count({0, p})) {
            continue;
        }
        visited.insert({0, p});

        if (p.pos == target) {
            //std::cout << score << std::endl;
            //ASSERT(score == get_env().get_dist(source, target), "invalid score");
            return score;
        }

        auto step = [&](Action action) {
            Position to = p.simulate_action(action);
            if (!to.is_valid()) {
                return;
            }
            if (visited.count({0, to})) {
                return;
            }
            S.insert({score + 1 + get_collisions(std::min(k, PLANNER_DEPTH - 1), p, action).size() * collision_power
                      - get_env().get_dist(p, target)
                      + get_env().get_dist(to, target),
                      k + 1, to});
        };

        step(Action::FW);
        step(Action::CR);
        step(Action::CCR);
        step(Action::W);
    }

    //std::cout << 0 << std::endl;
    //std::cout << get_env().get_dist(source, target) << std::endl;
    FAILED_ASSERT("unable to find path");
    return 0;
}

void PlannerMachine::process_path(uint32_t r, int32_t expected, int32_t value, int32_t sign) {
    auto &robot = robots[r];
    Position p = robot.start;
    std::array<int32_t, PLANNER_DEPTH + 1> dists{};
    dists[0] = get_env().get_dist(robot.start, robot.target);
    for (uint32_t k = 0; k < PLANNER_DEPTH; k++) {
        auto old = p;
        p = p.simulate_action(robot.actions[k]);
        ASSERT(0 <= p.pos && p.pos < map[k].size(), "invalid pos: " + std::to_string(p.pos));
        ASSERT(map[k][p.pos] == expected, "invalid map: " + std::to_string(map[k][p.pos]));
        map[k][p.pos] = value;

        dists[k + 1] = get_env().get_dist(p, robot.target);

        if (robot.actions[k] == Action::FW) {
            int pos = old.pos;
            int to = p.pos;
            if (pos > to) {
                std::swap(pos, to);
            }
            if (to - pos == 1) {
                // gor
                ASSERT(map_gor[k][pos] == expected, "invalid map");
                map_gor[k][pos] = value;
            } else {
                // ver
                ASSERT(to - pos == get_env().get_cols(), "invalid pos and to");
                ASSERT(map_ver[k][pos] == expected, "invalid map");
                map_ver[k][pos] = value;
            }
        }
    }

    for (uint32_t k = 0; k < PLANNER_DEPTH; k++) {
        solution_info.sum_dist_change[k] += sign * (dists[k] - dists[k + 1]);
    }
}

void PlannerMachine::add_path(uint32_t r) {
    process_path(r, -1, r, +1);
}

void PlannerMachine::remove_path(uint32_t r) {
    process_path(r, r, -1, -1);
}

double PlannerMachine::get_score(const SolutionInfo &info) const {
    constexpr double r = 0.90;

    const double norm = 1 / static_cast<double>(robots.size());

    double res = 0;
    double m = 1;
    for (uint32_t k = 0; k < PLANNER_DEPTH; k++) {
        res += info.sum_dist_change[k] * m;
        m *= r;
    }
    return res;
}

SolutionInfo PlannerMachine::get_solution_info() const {
    return solution_info;
}

bool PlannerMachine::compare(SolutionInfo old, SolutionInfo cur, Randomizer &rnd) {
    return get_score(old) <= get_score(cur);
}

PlannerMachine::PlannerMachine() {
    robots.resize(get_env().get_shared_env().num_of_agents);
    for (uint32_t k = 0; k < PLANNER_DEPTH; k++) {
        map[k].resize(get_env().get_size(), -1);
        map_gor[k].resize(get_env().get_size(), -1);
        map_ver[k].resize(get_env().get_size(), -1);
    }
    for (uint32_t r = 0; r < robots.size(); r++) {
        robots[r].start = Position(get_env().get_shared_env().curr_states[r].location,
                                   get_env().get_shared_env().curr_states[r].orientation);
        add_path(r);
    }
}

bool PlannerMachine::try_remove_and_add(Randomizer &rnd) {
    uint32_t r1 = rnd.get(0, robots.size() - 1);
    uint32_t r2 = rnd.get(0, robots.size() - 1);

    if (r1 == r2) {
        return false;
    }

    auto old = get_solution_info();

    auto r1a = robots[r1].actions;
    auto r2a = robots[r2].actions;

    remove_path(r1);
    remove_path(r2);

    auto act = get_path(r2);
    if (!act) {
        add_path(r1);
        add_path(r2);
        return false;
    }
    robots[r2].actions = *act;
    add_path(r2);

    act = get_path(r1);

    if (!act) {
        remove_path(r2);
        robots[r2].actions = r2a;
        add_path(r1);
        add_path(r2);
        return false;
    }

    robots[r1].actions = *act;
    add_path(r1);

    return consider(old, rnd, [&]() {
        remove_path(r1);
        remove_path(r2);
        robots[r1].actions = r1a;
        robots[r2].actions = r2a;
        add_path(r1);
        add_path(r2);
    });
}

bool PlannerMachine::try_crack(Randomizer &rnd) {
    uint32_t r = rnd.get(0, robots.size() - 1);

    auto old = get_solution_info();
    auto old_action = robots[r].actions;

    remove_path(r);

    auto [rids, action] = get_collision_path(r, rnd.get_d(0, 5));

    std::vector<Actions> old_actions(rids.size());
    for (uint32_t k = 0; k < rids.size(); k++) {
        old_actions[k] = robots[rids[k]].actions;
        remove_path(rids[k]);
    }

    robots[r].actions = action;
    add_path(r);

    for (uint32_t k = 0; k < rids.size(); k++) {
        auto action = get_path(rids[k]);
        if (!action) {
            for (uint32_t d = 0; d < k; d++) {
                remove_path(rids[d]);
            }
            remove_path(r);

            for (uint32_t d = 0; d < rids.size(); d++) {
                robots[rids[d]].actions = old_actions[d];
                add_path(rids[d]);
            }
            robots[r].actions = old_action;
            add_path(r);
            return false;
        }
        ASSERT(action, "is nullopt");
        robots[rids[k]].actions = *action;
        add_path(rids[k]);
    }

    return consider(old, rnd, [&]() {
        for (uint32_t r: rids) {
            remove_path(r);
        }
        remove_path(r);

        robots[r].actions = old_action;
        add_path(r);
        for (uint32_t k = 0; k < rids.size(); k++) {
            robots[rids[k]].actions = old_actions[k];
            add_path(rids[k]);
        }
    });
}

bool PlannerMachine::try_move_over(uint32_t k, uint32_t r, Randomizer &rnd) {
    Position p = robots[r].start;
    for (uint32_t d = 0; d < k; d++) {
        p = p.simulate_action(robots[r].actions[d]);
    }
    // стоим сейчас в p
    // нам нужно отойти с этой позиции

    // рассмотрим парочку поворотов и move forward

    // проверим, что они корректны
    // для каждой корректной штуки мы вызовем рекурсивно try_move_over
    // для робота того, кто нам мешает в move_forward
    FAILED_ASSERT("TODO");
    return false;
}

bool PlannerMachine::try_move_over(Randomizer &rnd) {
    uint32_t r = rnd.get(0, robots.size() - 1);

    auto old = get_solution_info();

    return try_move_over(0, r, rnd);

    /*return consider(old, rnd, [&]() {
        for (uint32_t r: rids) {
            remove_path(r);
        }
        remove_path(r);

        robots[r].actions = old_action;
        add_path(r);
        for (uint32_t k = 0; k < rids.size(); k++) {
            robots[rids[k]].actions = old_actions[k];
            add_path(rids[k]);
        }
    });*/
}

void PlannerMachine::run(TimePoint end_time) {
    update_targets();
    Randomizer rnd;//(clock());
    for (uint32_t step = 0;; step++) {
        if (step % 1 == 0) {
            if (std::chrono::steady_clock::now() >= end_time) {
                break;
            }
        }

        /*uint32_t r = 0;
        remove_path(r);
        auto action = get_path(r);
        if(!action){
            add_path(r);
            continue;
        }
        robots[r].actions = *action;
        add_path(r);*/

        try_remove_and_add(rnd);
        //try_crack(rnd);
        //try_move_over(rnd);
    }
}

void PlannerMachine::set_plan(std::vector<Action> &plan) const {
    ASSERT(robots.size() == plan.size(), "invalid sizes");
    for (uint32_t r = 0; r < robots.size(); r++) {
        plan[r] = robots[r].actions[0];
    }
}