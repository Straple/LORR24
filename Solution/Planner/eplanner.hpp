#pragma once

#include <Objects/Basic/time.hpp>

#include <ActionModel.h>
#include <SharedEnv.h>

class EPlanner {
    SharedEnvironment *env;
public:
    explicit EPlanner(SharedEnvironment *env);

    EPlanner();

    std::vector<Action> plan(TimePoint end_time);
};
