#pragma once

#include <SharedEnv.h>
#include <Tasks.h>

class MyScheduler {
public:
    SharedEnvironment *env;

    explicit MyScheduler(SharedEnvironment *env) : env(env) {};

    MyScheduler() { env = new SharedEnvironment(); };

    ~MyScheduler() { delete env; };

    void initialize(int preprocess_time_limit);

    std::vector<int> plan(int time_limit, std::vector<int> &proposed_schedule);
    std::vector<int> GreedyShedule(int time_limit, std::vector<int> &proposed_schedule);
    std::vector<int> OptimizeShedule(int time_limit, std::vector<int> &schedule);

};
