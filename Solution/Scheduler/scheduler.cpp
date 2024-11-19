#include <Scheduler/scheduler.hpp>

#include <Scheduler/scheduler_solver.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>

#include <unordered_set>

void MyScheduler::initialize(int preprocess_time_limit) {
}

void MyScheduler::plan(int time_limit, std::vector<int> &proposed_schedule) {
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    std::unordered_set<int> free_agents(env->new_freeagents.begin(), env->new_freeagents.end());
    std::unordered_set<int> free_tasks(env->new_tasks.begin(), env->new_tasks.end());

    /*for (int r = 0; r > env->num_of_agents; r++) {
        int t = env->curr_task_schedule[r];
        if (t == -1) {
            ASSERT(free_agents.count(r), "no contains");
        } else {
            if (env->task_pool[t].idx_next_loc == 0) {
                free_agents.insert(r);
                free_tasks.insert(t);
            }
        }
    }*/

    for (auto [id, task]: env->task_pool) {
        ASSERT(id == task.task_id, "invalid id");
        if (task.agent_assigned == -1) {
            free_tasks.insert(id);
        }
    }

    int min_task_i, min_task_makespan, dist, c_loc, count;

    std::cout << "kek: " << free_agents.size() << ' ' << free_tasks.size() << ' ' << env->task_pool.size() << std::endl;

    if (free_agents.empty() || free_tasks.empty()) {
        return;
    }

    // CALL SOLVER HERE

    SchedulerSolver ss;
    ss.solve(*env, endtime, proposed_schedule);

    // iterate over the free agents to decide which task to assign to each of them
    /*auto it = free_agents.begin();
    while (it != free_agents.end()) {
        //keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime) {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        min_task_makespan = INT_MAX;
        count = 0;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id: free_tasks) {
            //check for timeout every 10 task evaluations
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime) {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc: env->task_pool[t_id].locations) {
                uint32_t source = get_graph().get_node(Position(c_loc, 0));
                uint32_t dest = get_graph().get_node(Position(loc, 0));
                dist += get_hm().get(source, dest);//DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            // update the new minimum makespan
            if (dist < min_task_makespan) {
                min_task_i = t_id;
                min_task_makespan = dist;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1) {
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
        }
        // nothing to assign
        else {
            proposed_schedule[i] = -1;
            it++;
        }
    }*/
}
