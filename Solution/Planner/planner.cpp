#include "planner.hpp"

#include "../assert.hpp"
#include "planner_solver.hpp"

int EPlanner::get_target(int r) const {
    int task_id = env->curr_task_schedule[r];
    if (task_id == -1) {
        return -1;
    }
    int t = 0;
    for (; t < env->task_pool.size() && task_id != env->task_pool[t].task_id; t++) {}
    ASSERT(t < env->task_pool.size() && env->task_pool[t].task_id == task_id, "invalid t");

    auto task = env->task_pool[t];
    int target = task.get_next_loc();
    ASSERT(0 <= target && target < env->cols * env->rows, "invalid target: " + std::to_string(target));

    return target;
}

EPlanner::EPlanner(SharedEnvironment *env) : env(env) {}

EPlanner::EPlanner() {
    env = new SharedEnvironment();
}

void EPlanner::initialize(int preprocess_time_limit) {
    std::vector<bool> map(env->map.size());
    ASSERT(env->map.size() == env->cols * env->rows, "invalid map size");
    for (int pos = 0; pos < map.size(); pos++) {
        map[pos] = env->map[pos] == 0;
    }
    PlannerSolver solver(env->rows, env->cols, map, {},{}, 42);
    solver.build_dist();
}

// return next states for all agents
void EPlanner::plan(int time_limit, std::vector<Action> &plan) {
    plan.resize(env->num_of_agents, Action::W);

    time_limit -= std::chrono::duration_cast<milliseconds>(
            std::chrono::steady_clock::now() - env->plan_start_time).count();
    time_limit *= 0.5;

    auto finish_time = std::chrono::steady_clock::now();
    finish_time += std::chrono::milliseconds(time_limit);


    std::vector<bool> map(env->map.size());
    ASSERT(env->map.size() == env->cols * env->rows, "invalid map size");
    for (int pos = 0; pos < map.size(); pos++) {
        map[pos] = env->map[pos] == 0;
    }
    std::vector<Position> robots_pos(env->num_of_agents);
    std::vector<int> robots_target(env->num_of_agents);
    for (int r = 0; r < robots_pos.size(); r++) {
        robots_pos[r] = Position(env->curr_states[r].location, env->curr_states[r].orientation, env);
        robots_target[r] = get_target(r);
    }
    PlannerSolver solver(env->rows, env->cols, map, robots_pos, robots_target, 42);
    solver.run(time_limit);
    auto [solution_info, actions] = solver.get();
    for (uint32_t r = 0; r < actions.size(); r++) {
        plan[r] = actions[r];
    }
}
