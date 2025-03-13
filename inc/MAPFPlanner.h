#pragma once

#include <ActionModel.h>
#include <SharedEnv.h>

class MAPFPlanner {
public:
    SharedEnvironment *env;

    explicit MAPFPlanner(SharedEnvironment *env) : env(env) {}

    MAPFPlanner() {
        env = new SharedEnvironment();
    }

    virtual ~MAPFPlanner() {
        delete env;
    }

    virtual void initialize(int preprocess_time_limit);

    virtual void plan(int time_limit, std::vector<Action> &plan);
};
