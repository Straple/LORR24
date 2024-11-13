#include "scheduler.hpp"

#include "../Objects/environment.hpp"
#include "../Objects/assert.hpp"

void MyScheduler::initialize(int preprocess_time_limit) {

}

void MyScheduler::plan(int time_limit, std::vector<int> &proposed_schedule) {
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);

    ASSERT(env->curr_task_schedule.size() == env->num_of_agents, "invalid sizes");
    proposed_schedule = env->curr_task_schedule;

    std::vector<int> robot_task_id(env->num_of_agents, -1);
    for (uint32_t i = 0; i < env->task_pool.size(); i++) {
        auto &task = env->task_pool[i];
        if (task.agent_assigned != -1) {
            ASSERT(0 <= task.agent_assigned && task.agent_assigned < env->num_of_agents, "invalid agent assigned");
            robot_task_id[task.agent_assigned] = i;
        }
    }

    std::vector<int> free_agents;
    for (int i = 0; i < env->num_of_agents; i++) {
        int my_id = robot_task_id[i];
        if (proposed_schedule[i] == -1 ||
                (env->task_pool[my_id].idx_next_loc == 0 && env->curr_states[i].location != env->task_pool[my_id].locations[0])) {
            free_agents.push_back(i);
        }
    }

    for (int r: free_agents) {
        if (std::chrono::steady_clock::now() >= endtime) {
            break;
        }
        int my_id = robot_task_id[r];
        if (my_id != -1) {
            ASSERT(env->task_pool[my_id].agent_assigned == r, "blya");
            env->task_pool[my_id].agent_assigned = -1;
            proposed_schedule[r] = -1;
        }
        int min_task_i = -1;
        int min_task_makespan = INT_MAX;
        for (int i_task = 0; i_task < env->task_pool.size() && std::chrono::steady_clock::now() < endtime; i_task++) {

            if (env->task_pool[i_task].agent_assigned != -1)
                continue;
            int dist = 0;
            int c_loc = env->curr_states.at(r).location;
            for (int loc: env->task_pool[i_task].locations) {
                dist += get_env().get_dist(Position(c_loc, 0), loc);
                c_loc = loc;
            }
            if (dist < min_task_makespan) {
                min_task_i = i_task;
                min_task_makespan = dist;
            }
        }

        if (min_task_i != -1) {
            proposed_schedule[r] = env->task_pool[min_task_i].task_id;
            env->task_pool[min_task_i].agent_assigned = r;
        }
    }
    // cout << ((float)(clock() - start))/CLOCKS_PER_SEC << endl;
}
