#pragma once

#include <Objects/Basic/position.hpp>
#include <Objects/Basic/time.hpp>
#include <Objects/Environment/robot_handler.hpp>

// Priority Inheritance with BackTracking
class PIBT {
    TimePoint end_time;

    const std::vector<Robot> &robots;

    std::vector<uint32_t> desires;

    std::unordered_map<uint32_t, uint32_t> pos_to_robot;

    bool build(uint32_t r, int banned_desired, uint32_t depth);

public:
    PIBT(const std::vector<Robot> &robots, TimePoint end_time);

    void solve();

    [[nodiscard]] std::vector<Action> get_actions() const;
};
