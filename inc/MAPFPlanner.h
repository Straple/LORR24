#pragma once
#include "ActionModel.h"
#include "SharedEnv.h"
#include <ctime>


class MAPFPlanner {
public:
    SharedEnvironment *env;

    MAPFPlanner(SharedEnvironment *env) : env(env){};
    MAPFPlanner() { env = new SharedEnvironment(); };
    virtual ~MAPFPlanner() { delete env; };


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> &plan);
};
