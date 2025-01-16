#pragma once

#include <ActionModel.h>
#include <SharedEnv.h>

class EPlanner {
    SharedEnvironment *env;
public:
    explicit EPlanner(SharedEnvironment *env);

    EPlanner();

    void plan(int time_limit, std::vector<Action> &plan);
};
