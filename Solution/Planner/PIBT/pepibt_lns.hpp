#pragma once

#include <Objects/Basic/time.hpp>
#include <Objects/Environment/robot_handler.hpp>

#include <ActionModel.h>

// Parallel Enhanced Priority Inheritance with BackTracking + Large Neighborhood Search
class PEPIBT_LNS {

    TimePoint end_time;

    const std::vector<Robot> &robots;

    std::vector<Action> actions;

public:
    PEPIBT_LNS(const std::vector<Robot> &robots, TimePoint end_time);

    void solve(uint64_t seed);

    [[nodiscard]] std::vector<Action> get_actions() const;
};