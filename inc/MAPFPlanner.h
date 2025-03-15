#pragma once

#include <ActionModel.h>
#include <SharedEnv.h>

#include "../Solution2/inc/MAPFPlanner.h"

class MAPFPlanner {
public:
    SharedEnvironment *env;

    SmartMAPFPlanner smart_planner;

    explicit MAPFPlanner(SharedEnvironment *env) : env(env), smart_planner(env) {}

    MAPFPlanner() {
        env = new SharedEnvironment();
    }

    virtual ~MAPFPlanner() {
        delete env;
    }

    virtual void initialize(int preprocess_time_limit);

    virtual void plan(int time_limit, std::vector<Action> &plan);
};
