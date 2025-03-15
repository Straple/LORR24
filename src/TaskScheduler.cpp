#include <TaskScheduler.h>

#include <const.h>
#include <scheduler.h>

#include <Objects/Basic/time.hpp>
#include <Objects/Environment/environment.hpp>

#include <settings.hpp>

/**
 * Initializes the task scheduler with a given time limit for preprocessing.
 * 
 * This function prepares the task scheduler by allocating up to half of the given preprocessing time limit 
 * and adjust for a specified tolerance to account for potential timing errors. 
 * It ensures that initialization does not exceed the allocated time.
 * 
 * @param preprocess_time_limit The total time limit allocated for preprocessing (in milliseconds).
 *
 */
void TaskScheduler::initialize(int preprocess_time_limit) {
#ifdef ENABLE_DEFAULT_SCHEDULER
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = preprocess_time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);
#else
    init_environment(*env);
#endif
}

/**
 * Plans a task schedule within a specified time limit.
 * 
 * This function schedules tasks by calling shedule_plan function in default planner with half of the given time limit,
 * adjusted for timing error tolerance. The planned schedule is output to the provided schedule vector.
 * 
 * @param time_limit The total time limit allocated for scheduling (in milliseconds).
 * @param proposed_schedule A reference to a vector that will be populated with the proposed schedule (next task id for each agent).
 */

void TaskScheduler::plan(int time_limit, std::vector<int> &proposed_schedule) {
    ETimer timer;
#ifdef ENABLE_DEFAULT_SCHEDULER
    //give at most half of the entry time_limit to scheduler;
    //-SCHEDULER_TIMELIMIT_TOLERANCE for timing error tolerance
    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_plan(limit, proposed_schedule, env);

    PRINT(
            uint32_t cnt = 0;
            for (uint32_t r = 0; r < proposed_schedule.size(); r++) {
                cnt += proposed_schedule[r] != -1;
            };
            uint32_t p = cnt * 100.0 / proposed_schedule.size();
            ASSERT(0 <= p && p <= 100, "invalid p: " + std::to_string(p));
            Printer() << "[Dummy Scheduler] assigned robots: " << p << "%" << (p != 100 ? "bad\n" : "\n");
            Printer() << "[Dummy Scheduler] time: " << timer << '\n';);
#else
    TimePoint end_time = std::min(env->plan_start_time + Milliseconds(time_limit - 10), get_now() + Milliseconds(SCHEDULER_REBUILD_DP_TIME + SCHEDULER_LAZY_SOLVE_TIME + SCHEDULER_LNS_SOLVE_TIME));
    update_environment(*env);
    my_scheduler.plan(end_time, proposed_schedule);

    PRINT(
            uint32_t cnt = 0;
            for (uint32_t r = 0; r < proposed_schedule.size(); r++) {
                cnt += proposed_schedule[r] != -1;
            };
            uint32_t p = cnt * 100.0 / proposed_schedule.size();
            ASSERT(0 <= p && p <= 100, "invalid p: " + std::to_string(p));
            Printer() << "[Scheduler] assigned robots: " << p << "%" << (p != 100 ? "bad\n" : "\n");
            Printer() << "[Scheduler] time: " << timer << '\n';);
#endif
}
