#pragma once

#include <Scheduler/scheduler_solver.hpp>

#include <SharedEnv.h>
#include <Tasks.h>

class MyScheduler {

    SharedEnvironment *env = nullptr;

    SchedulerSolver solver;

public:
    MyScheduler() = default;

    explicit MyScheduler(SharedEnvironment *env);

    std::vector<int> plan(int time_limit, std::vector<int> &proposed_schedule);

    std::vector<int> solver_schedule(int time_limit, std::vector<int> &proposed_schedule);

    std::vector<int> greedy_schedule(int time_limit, std::vector<int> &proposed_schedule);

    std::vector<int> greedy_schedule_double(int time_limit, std::vector<int> &proposed_schedule);

    std::vector<int> artem_schedule(int time_limit, std::vector<int> &proposed_schedule);
};
