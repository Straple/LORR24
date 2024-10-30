#include "eplanner.hpp"

#include "../assert.hpp"

/*void EPlanner::build_path(int robot, int target) {
    if (!robots[robot].path.empty()) {
        if (robots[robot].path.back().second == target) {
            return;// уже есть путь
        } else {
            remove_path(robot);// устаревший путь
        }
    }

    struct State {
        int time;
        Position pos;
    };

    vector<State> Q0, Q1;
    Q0.push_back({timestamp, robots[robot].position});

    struct my_cmp {
        bool operator()(State lhs, State rhs) const {
            if (lhs.time != rhs.time) {
                return lhs.time < rhs.time;
            }
            return lhs.pos < rhs.pos;
        }
    };

    std::set<State, my_cmp> visited;
    visited.insert(State{timestamp, robots[robot].position});

    while (!Q0.empty() || !Q1.empty()) {
        if (Q0.empty()) {
            std::swap(Q0, Q1);
        }

        auto [time, pos] = Q0.back();
        Q0.pop_back();

        ASSERT(pos.is_valide(env), "p is invalid");

        if (pos == target) {
            return d;
        }

        visited.insert(p);

#define STEP(init)                                                  \
    {                                                               \
        Position q = (init);                                        \
        if (q.is_valide(env) && visited.find(q) == visited.end()) { \
            visited.insert(q);                                      \
            Q1.push_back(q);                                        \
        }                                                           \
    }

        STEP(p.move_forward(env));
        STEP(p.rotate());
        STEP(p.counter_rotate());

#undef STEP
    }

    ASSERT(false, "not found path");
    return -1;
}

void EPlanner::remove_path(int robot) {
    for (auto [t, pos]: robots_paths[robot]) {
        ASSERT(plan_map[t][pos] == robot, "unable to remove path");
        plan_map[t][pos] = -1;
    }
    robots_paths[robot].clear();
}*/

[[nodiscard]] int EPlanner::get_target(int r) const {
    int task_id = env->curr_task_schedule[r];
    if (task_id == -1) {
        return -1;
    }
    int t = 0;
    for (; t < env->task_pool.size() && task_id != env->task_pool[t].task_id; t++) {}
    ASSERT(t < env->task_pool.size() && env->task_pool[t].task_id == task_id, "invalid t");

    auto task = env->task_pool[t];
    int target = task.get_next_loc();
    //ASSERT(0 <= target && target < env->cols * env->rows, "invalid target: " + std::to_string(target));

    if (target == robots[r].position.pos) {
        //ASSERT(false, "WTF?");
        return -1;// TODO: робот же уже стоит в этой позиции???
    }
    return target;
}

int EPlanner::find_robot(int pos) const {
    vector<int> ans;
    for (int r = 0; r < robots.size(); r++) {
        if (robots[r].position.simulate_action(robots[r].action, env).pos == pos) {
            ans.push_back(r);
        }
    }

    if (ans.empty()) {
        return -1;
    } else {
        ASSERT(ans.size() == 1, "invalid robots");
        return ans[0];
    }
}

bool EPlanner::move_over(int r) {
    if (robots[r].action != Action::W) {
        return false;
    }
    // ASSERT(robots[r].action == Action::W, "unable to move over robot");
    // найдем куда можно отойти роботу

    robots[r].action = Action::NA;

    {
        auto to = robots[r].position.simulate_action(Action::FW, env);
        if (to.is_valide(env)) {
            int to_r = find_robot(to.pos);
            if (to_r == -1) {
                // можем пойти вперед, в незанятое место
                robots[r].action = Action::FW;
                return true;
            } else if (move_over(to_r)) {
                if (find_robot(to.pos) == -1) {
                    robots[r].action = Action::FW;
                } else {
                    // этот робот пока там
                    robots[r].action = Action::W;
                }
                return true;
            }
        }
    }

    {
        auto to = robots[r].position.simulate_action(Action::CR, env);
        to = to.simulate_action(FW, env);
        if (to.is_valide(env)) {
            int to_r = find_robot(to.pos);
            if (to_r == -1 || move_over(to_r)) {
                robots[r].action = Action::CR;
                return true;
            }
        }
    }

    {
        auto to = robots[r].position.simulate_action(Action::CCR, env);
        to = to.simulate_action(FW, env);
        if (to.is_valide(env)) {
            int to_r = find_robot(to.pos);
            if (to_r == -1 || move_over(to_r)) {
                robots[r].action = Action::CCR;
                return true;
            }
        }
    }

    robots[r].action = Action::W;
    return false;
}

void EPlanner::plan_robot(int r) {
    if (get_target(r) == -1) {
        return;
    }

    if (robots[r].action != Action::W) {
        return;// уже посчитан
    }

    auto pos = robots[r].position;
    Action best_a = Action::W;
    int best_d = 1e9;

    {
        auto to = pos.simulate_action(Action::W, env);
        if (to.is_valide(env)) {
            int d = dist_machine.get_dist(to, get_target(r), env);
            if (d < best_d) {
                best_d = d;
                best_a = Action::W;
            }
        }
    }

    {
        auto to = pos.simulate_action(Action::CR, env);
        if (to.is_valide(env)) {
            int d = dist_machine.get_dist(to, get_target(r), env);
            if (d < best_d) {
                best_d = d;
                best_a = Action::CR;
            }
        }
    }

    {
        auto to = pos.simulate_action(Action::CCR, env);
        if (to.is_valide(env)) {
            int d = dist_machine.get_dist(to, get_target(r), env);
            if (d < best_d) {
                best_d = d;
                best_a = Action::CCR;
            }
        }
    }

    {
        auto to = pos.simulate_action(Action::FW, env);
        if (to.is_valide(env)) {
            int d = dist_machine.get_dist(to, get_target(r), env);
            if (d < best_d) {

                int to_r = find_robot(to.pos);
                if (to_r != -1) {
                    // есть мазафака, который преградил нам путь

                    if (robots[to_r].action == Action::W) {
                        move_over(to_r);
                    } else {
                        // у него уже есть какое-то действие, не будем его трогать
                    }
                } else {
                    best_d = d;
                    best_a = Action::FW;
                }
            }
        }
    }

    robots[r].action = best_a;
}

void EPlanner::flush_robots() {
    for (int r = 0; r < robots.size(); r++) {
        robots[r].position = robots[r].position.simulate_action(robots[r].action, env);
        robots[r].action = Action::W;
    }
}

EPlanner::EPlanner(SharedEnvironment *env) : env(env) {}
EPlanner::EPlanner() {
    env = new SharedEnvironment();
}

void EPlanner::initialize(int preprocess_time_limit) {
    robots.resize(env->num_of_agents);
}

// return next states for all agents
void EPlanner::plan(int time_limit, std::vector<Action> &plan) {
    plan.resize(env->num_of_agents, Action::W);

    static bool first = true;
    if (first) {
        first = false;
        for (int r = 0; r < robots.size(); r++) {
            //ASSERT(robots[r].position == Position(env->curr_states[r].location, env->curr_states[r].orientation, env), "invalid robot");
            robots[r].position = Position(env->curr_states[r].location, env->curr_states[r].orientation, env);
        }
    } else {
        for (int r = 0; r < robots.size(); r++) {
            ASSERT(robots[r].position == Position(env->curr_states[r].location, env->curr_states[r].orientation, env), "invalid robot");
        }
    }

    /*std::cout << "\ntarget: " << get_target(0) << std::endl;
    if (get_target(0) == -1) {
        get_target(0);
    }*/

    time_limit -= std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count();
    time_limit *= 0.5;

    auto finish_time = std::chrono::steady_clock::now();
    finish_time += std::chrono::milliseconds(time_limit);

#define PROCESS_TIME                                      \
    if (std::chrono::steady_clock::now() > finish_time) { \
        return;                                           \
    }
    // time_limit = время выполнения планировки в ms

    for (int r = 0; r < env->num_of_agents; r++) {
        plan_robot(r);
    }

    for (int r = 0; r < env->num_of_agents; r++) {
        plan[r] = robots[r].action;
    }
    flush_robots();
}
