#include "Entry.h"
#include "Tasks.h"
#include "heuristics.h"
#include "utils.h"

#include <Objects/Basic/time.hpp>
#include <Objects/Environment/environment.hpp>

// The initialize function will be called by competition system at the preprocessing stage.
// Implement the initialize functions of the planner and scheduler to load or compute auxiliary data.
// Note that, this function runs untill preprocess_time_limit (in milliseconds) is reached.
// This is an offline step, after it completes then evaluation begins.
void Entry::initialize(int preprocess_time_limit) {
    scheduler->initialize(preprocess_time_limit);
    planner->initialize(preprocess_time_limit);
}

//The compute function will be called by competition system on each timestep.
//It computes:
//  1. a schedule that specifies which agent complete which task.
//  2. a next action that specifies how each agent should move in the next timestep.
//NB: the parameter time_limit is specified in milliseconds.
void Entry::compute(int time_limit, std::vector<Action> &plan, std::vector<int> &proposed_schedule) {
#ifdef ENABLE_PRINT_LOG
    static Timer total_timer;
    Timer timer;
    Printer() << "\n";
    Printer() << "Timestep: " << env->curr_timestep << '\n';
    // прошло больше 150 секунд, это плохое решение
    if (total_timer.get_ms() > 150'000) {
        Printer() << "bad solution\n";
        return;
    }
#endif

    update_environment(*env);

    //call the task scheduler to assign tasks to agents
    scheduler->plan(time_limit, proposed_schedule);

#ifdef ENABLE_PRINT_LOG
    {
        uint32_t cnt = 0;
        for (uint32_t r = 0; r < proposed_schedule.size(); r++) {
            cnt += proposed_schedule[r] != -1;
        }
        Printer() << "Scheduler robots init: " << cnt << "/" << proposed_schedule.size() << " ("
                  << cnt * 100.0 / proposed_schedule.size() << "%)\n";
    }
#endif

    //then update the first unfinished errand/location of tasks for planner reference
    update_goal_locations(proposed_schedule);

    //call the planner to compute the actions
    planner->plan(time_limit, plan);

#ifdef ENABLE_PRINT_LOG
    Printer() << "Entry time: " << timer << '\n';
    Printer() << "Total time: " << total_timer << '\n';
#endif
}

// Set the next goal locations for each agent based on the proposed schedule
void Entry::update_goal_locations(std::vector<int> &proposed_schedule) {
    // record the proposed schedule so that we can tell the planner
    env->curr_task_schedule = proposed_schedule;

    // The first unfinished errand/location of each task is the next goal for the assigned agent.
    for (size_t i = 0; i < proposed_schedule.size(); i++) {
        env->goal_locations[i].clear();
        int t_id = proposed_schedule[i];
        if (t_id == -1)
            continue;

        int i_loc = env->task_pool[t_id].idx_next_loc;
        env->goal_locations[i].push_back(
                {env->task_pool[t_id].locations.at(i_loc), env->task_pool[t_id].t_revealed});
    }
}
