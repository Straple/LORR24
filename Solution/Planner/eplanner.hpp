#pragma once

#include <ActionModel.h>
#include <SharedEnv.h>

class EPlanner {
public:
    SharedEnvironment *env;

    explicit EPlanner(SharedEnvironment *env);
    EPlanner();

    virtual void initialize(int preprocess_time_limit);

    virtual void plan(int time_limit, std::vector<Action> &plan);
};
