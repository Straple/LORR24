#pragma once

#include "ActionModel.h"
#include "MAPFPlanner.h"
#include "SharedEnv.h"
#include "TaskScheduler.h"
#include <ctime>

#include "../Solution/Planner/eplanner.hpp"
#include "../Solution/Scheduler/scheduler.hpp"
#include "../Solution/settings.hpp"

class Entry {
public:
    SharedEnvironment *env;
    PLANNER *planner;
    TASKSHEDULLER *scheduler;

    Entry(SharedEnvironment *env) : env(env) {
        planner = new PLANNER(env);
    };

    Entry() {
        env = new SharedEnvironment();
        planner = new PLANNER(env);
        scheduler = new TASKSHEDULLER(env);
    };

    virtual ~Entry() { delete env; };

    virtual void initialize(int preprocess_time_limit);

    // return next actions and the proposed task schedule for all agents
    virtual void compute(int time_limit, std::vector<Action> &plan, std::vector<int> &proposed_schedule);

    void update_goal_locations(std::vector<int> &proposed_schedule);
};