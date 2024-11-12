#include "planner_machine.hpp"

#include "../Objects/environment.hpp"

bool PlannerMachine::is_allowed(int k, Position p, Action action) const {
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

void PlannerMachine::update_targets() {
    for (auto &robot: robots) {
        robot.target = -1;
    }

    for (auto &task: get_env().get_shared_env().task_pool) {
        if (task.agent_assigned != -1) {
            ASSERT(0 <= task.agent_assigned && task.agent_assigned < robots.size(), "invalid task agent assigned");
            robots[task.agent_assigned].target = task.get_next_loc();
            ASSERT(0 <= robots[task.agent_assigned].target && robots[task.agent_assigned].target < get_env().get_size(), "invalid target");
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
        Actions path;
        Position p;
    };

    vector<State> Q0, Q1;
    // время и позиция
    std::set<std::pair<uint32_t, Position>> visited;

    Q0.push_back(State{get_wactions(), robots[r].start});

    Actions best_path;
    double best_dist = 1e300;

    uint32_t d = 0;
    while (!Q0.empty() || !Q1.empty()) {
        if (Q0.empty()) {
            std::swap(Q0, Q1);
            d++;
        }

        ASSERT(!Q0.empty(), "Q0 is empty");
        auto [path, p] = Q0.back();
        Q0.pop_back();

        ASSERT(p.is_valid(), "p is invalid");

        if (d == PLANNER_DEPTH) {
            double dist = get_env().get_dist(p, robots[r].target);
            Position v = robots[r].start;
            for (uint32_t k = 0; k < PLANNER_DEPTH; k++) {
                Position to = v.simulate_action(path[k]);
                //dist -= (get_env().get_dist(v, robots[r].target) - get_env().get_dist(to, robots[r].target)) * (1.0 / (k + 1));
                v = to;
            }
            ASSERT(v == p, "invalid path");

            if (dist <= best_dist) {
                best_dist = dist;
                best_path = path;
            }
            continue;
        }

#define STEP(action)                                               \
    {                                                              \
        Position to = p.simulate_action(action);                   \
        if (is_allowed(d, p, action) && !visited.count({d, to})) { \
            visited.insert({d, to});                               \
            path[d] = action;                                      \
            Q1.push_back({path, to});                              \
        }                                                          \
    }

        STEP(Action::W)
        STEP(Action::FW)
        STEP(Action::CR)
        STEP(Action::CCR)

#undef STEP
    }

    if (best_dist <= 1e10) {
        return best_path;
    } else {
        return std::nullopt;
    }
}

void PlannerMachine::add_path(uint32_t r) {
    Position p = robots[r].start;
    for (uint32_t k = 0; k < PLANNER_DEPTH; k++) {
        auto old = p;
        p = p.simulate_action(robots[r].actions[k]);
        ASSERT(0 <= p.pos && p.pos < map[k].size(), "invalid pos: " + std::to_string(p.pos));
        ASSERT(map[k][p.pos] == -1, "invalid map: " + std::to_string(map[k][p.pos]));
        map[k][p.pos] = r;

        if (robots[r].actions[k] == Action::FW) {
            int pos = old.pos;
            int to = p.pos;
            if (pos > to) {
                std::swap(pos, to);
            }
            if (to - pos == 1) {
                // gor
                ASSERT(map_gor[k][pos] == -1, "invalid map");
                map_gor[k][pos] = r;
            } else {
                // ver
                ASSERT(to - pos == get_env().get_cols(), "invalid pos and to");

                ASSERT(map_ver[k][pos] == -1, "invalid map");
                map_ver[k][pos] = r;
            }
        }
    }
}

void PlannerMachine::remove_path(uint32_t r) {
    Position p = robots[r].start;
    for (uint32_t k = 0; k < PLANNER_DEPTH; k++) {
        auto old = p;
        p = p.simulate_action(robots[r].actions[k]);
        ASSERT(0 <= p.pos && p.pos < map[k].size(), "invalid pos: " + std::to_string(p.pos));
        ASSERT(map[k][p.pos] == r, "invalid map: " + std::to_string(map[k][p.pos]));
        map[k][p.pos] = -1;

        if (robots[r].actions[k] == Action::FW) {
            int pos = old.pos;
            int to = p.pos;
            if (pos > to) {
                std::swap(pos, to);
            }
            if (to - pos == 1) {
                // gor
                ASSERT(map_gor[k][pos] == r, "invalid map");
                map_gor[k][pos] = -1;
            } else {
                // ver
                ASSERT(to - pos == get_env().get_cols(), "invalid pos and to");

                ASSERT(pos < map_ver[k].size(), "invalid pos");
                ASSERT(map_ver[k][pos] == r, "invalid map");
                map_ver[k][pos] = -1;
            }
        }
    }
}

PlannerMachine::PlannerMachine() {
    robots.resize(get_env().get_shared_env().num_of_agents);
    for (uint32_t k = 0; k < PLANNER_DEPTH; k++) {
        map[k].resize(get_env().get_size(), -1);
        map_gor[k].resize(get_env().get_size(), -1);
        map_ver[k].resize(get_env().get_size(), -1);
    }
    for (uint32_t r = 0; r < robots.size(); r++) {
        robots[r].start = Position(get_env().get_shared_env().curr_states[r].location, get_env().get_shared_env().curr_states[r].orientation);
        add_path(r);
    }
}

void PlannerMachine::try_remove_and_add(Randomizer &rnd) {
    uint32_t r1 = rnd.get(0, robots.size() - 1);
    uint32_t r2 = rnd.get(0, robots.size() - 1);

    if (r1 == r2) {
        return;
    }

    auto r1a = robots[r1].actions;
    auto r2a = robots[r2].actions;

    remove_path(r1);
    remove_path(r2);

    auto act = get_path(r2);
    if (!act) {
        ASSERT(false, "invalid");
        add_path(r1);
        add_path(r2);
        return;
    }
    robots[r2].actions = *act;
    add_path(r2);

    act = get_path(r1);

    if(!act){
        remove_path(r2);
        robots[r2].actions = r2a;
        add_path(r1);
        add_path(r2);
        return;
    }

    robots[r1].actions = *act;
    add_path(r1);
}

void PlannerMachine::run(TimePoint end_time) {
    update_targets();
    Randomizer rnd(clock());
    for (uint32_t step = 0;; step++) {
        if (step % 10 == 0) {
            if (std::chrono::steady_clock::now() >= end_time) {
                break;
            }
        }

        try_remove_and_add(rnd);
    }
}

void PlannerMachine::set_plan(std::vector<Action> &plan) const {
    ASSERT(robots.size() == plan.size(), "invalid sizes");
    for (uint32_t r = 0; r < robots.size(); r++) {
        plan[r] = robots[r].actions[0];
    }
}