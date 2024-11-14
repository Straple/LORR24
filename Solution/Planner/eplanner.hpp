#pragma once

#include "ActionModel.h"
#include "Machine/planner_machine.hpp"
#include "SharedEnv.h"

class EPlanner {
    PlannerMachine* planner_machine = nullptr;

    [[nodiscard]] int get_target(int r) const;

public:
    SharedEnvironment *env;

    explicit EPlanner(SharedEnvironment *env);
    EPlanner();

    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> &plan);
};
