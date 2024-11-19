#pragma once

#include <SharedEnv.h>
#include <Tasks.h>

class MyScheduler {
public:
    SharedEnvironment *env;

    MyScheduler(SharedEnvironment *env) : env(env){};

    MyScheduler() { env = new SharedEnvironment(); };

    virtual ~MyScheduler() { delete env; };

    virtual void initialize(int preprocess_time_limit);

    virtual void plan(int time_limit, std::vector<int> &proposed_schedule);
};
