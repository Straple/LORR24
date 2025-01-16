#pragma once

#include <ActionModel.h>
#include <SharedEnv.h>
#include <ctime>

#include <Planner/eplanner.hpp>

class MAPFPlanner {
public:
    SharedEnvironment *env;

    EPlanner eplanner;

    explicit MAPFPlanner(SharedEnvironment *env) : env(env), eplanner(env) {
    }

    MAPFPlanner() {
        env = new SharedEnvironment();
    }

    virtual ~MAPFPlanner() {
        delete env;
    }

    virtual void initialize(int preprocess_time_limit);

    virtual void plan(int time_limit, std::vector<Action> &plan);
};
