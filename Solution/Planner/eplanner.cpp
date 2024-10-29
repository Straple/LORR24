#include "eplanner.hpp"

#include "../assert.hpp"

EPlanner::EPlanner(SharedEnvironment *env) : env(env) {}
EPlanner::EPlanner() {
    env = new SharedEnvironment();
}

void EPlanner::initialize(int preprocess_time_limit) {
    // TODO
}

// return next states for all agents
void EPlanner::plan(int time_limit, std::vector<Action> &plan) {
    plan.resize(env->num_of_agents, Action::W);

    time_limit -= std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count();
    time_limit *= 0.95;

    static std::ofstream output("elog.txt");

    // time_limit = время выполнения планировки в ms

    if (plan.empty()) {
        return;
    }

    ASSERT(env->num_of_agents == plan.size(), std::to_string(env->num_of_agents) + " != " + std::to_string(plan.size()));

    for (int robot = 0; robot < env->num_of_agents; robot++) {
        plan[robot] = Action::W;
    }

    auto find_robot = [&](int pos) {
        vector<int> ans;
        for (int robot = 0; robot < env->num_of_agents; robot++) {
            if (env->curr_states[robot].location == pos) {
                ans.push_back(robot);
            }
        }
        ASSERT(ans.size() <= 1, "invalid ans");
        return ans;
    };

    output << "HELLO" << std::endl;

    for (int robot = 0; robot < env->num_of_agents; robot++) {
        int task = env->curr_task_schedule[robot];
        if (task != -1) {
            // у этого робота есть задача

            auto [goal_pos, reveal_time] = env->goal_locations[robot][0];

            // нужно робота в goal_pos
            // найдем куда ему лучше всего сейчас пойти

            Position p(env->curr_states[robot].location, env->curr_states[robot].orientation);

            ASSERT(p.pos != goal_pos, "why source is equal to target?");

            int best_d = 1e9;

#define STEP(to, action_type)                                                                                       \
    if ((to).is_valide(env) && find_robot(to.pos).empty() && dist_machine.get_dist((to), goal_pos, env) < best_d) { \
        best_d = dist_machine.get_dist((to), goal_pos, env);                                                        \
        plan[robot] = action_type;                                                                                  \
    }

            STEP(p.move_forward(env), Action::FW);
            STEP(p.rotate(), Action::CR);
            STEP(p.counter_rotate(), Action::CCR);

#undef STEP
        }
    }

    output << "BOO" << std::endl;

    // TODO: теперь нужно обработать полученные действия так, чтобы убрать плохие

    std::map<int, vector<int>> used;
    for (int robot = 0; robot < env->num_of_agents; robot++) {
        int task = env->curr_task_schedule[robot];
        if (task != -1) {
            Position p(env->curr_states[robot].location, env->curr_states[robot].orientation);
            if (plan[robot] == Action::FW) {
                p = p.move_forward(env);
            }

            if (!used[p.pos].empty()) {
                plan[robot] = Action::W;
            }
            used[p.pos].push_back(robot);
        }
    }
}
