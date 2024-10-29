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

    // проверяет, что нет робота, который хочет в эту позицию
    auto check_pos = [&](int pos, int robot_id) {
        vector<int> ans;
        for (int robot = 0; robot < env->num_of_agents; robot++) {
            if (robot == robot_id) {
                continue;
            }
            Position p(env->curr_states[robot].location, env->curr_states[robot].orientation, env);
            if (plan[robot] == Action::FW) {
                p = p.move_forward(env);
            } else if (plan[robot] == Action::CR) {
                p = p.rotate();
            } else if (plan[robot] == Action::CCR) {
                p = p.counter_rotate();
            } else if (plan[robot] == Action::W) {
                // wait
            } else {
                ASSERT(false, "invalid plan: " + std::to_string((int) plan[robot]));
            }
            if (p.pos == pos) {
                ans.push_back(robot);
            }
        }
        ASSERT(ans.size() <= 1, "invalid ans");
        return ans.empty();
    };

    for (int robot = 0; robot < env->num_of_agents; robot++) {
        int task_id = env->curr_task_schedule[robot];
        if (task_id != -1) {
            // у этого робота есть задача

            auto task = env->task_pool[task_id];
            //auto [goal_pos, reveal_time] =env->goal_locations[robot][task.idx_next_loc];
            int goal_pos = task.get_next_loc();

            // нужно робота в goal_pos
            // найдем куда ему лучше всего сейчас пойти

            Position p(env->curr_states[robot].location, env->curr_states[robot].orientation, env);

            if(p.pos == goal_pos){
                continue;
            }

            ASSERT(p.pos != goal_pos, "why source is equal to target? value: " + std::to_string(p.pos));

            int best_d = 1e9;
            Action best_action = Action::W;

#define STEP(to, action_type)                                                                                     \
    if ((to).is_valide(env) && check_pos(to.pos, robot) && dist_machine.get_dist((to), goal_pos, env) < best_d) { \
        best_d = dist_machine.get_dist((to), goal_pos, env);                                                      \
        best_action = action_type;                                                                                \
    }

            STEP(p.move_forward(env), Action::FW);
            STEP(p.rotate(), Action::CR);
            STEP(p.counter_rotate(), Action::CCR);

            plan[robot] = best_action;

#undef STEP
        }
    }
}
