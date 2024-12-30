#pragma once

#include <Objects/Environment/robot_handler.hpp>
#include <Planner/PIBT/pibt2.hpp>
#include <unordered_map>

constexpr static inline uint32_t PIBT_LNS_DEPTH = 3;

class PIBT_LNS {

    std::vector<Robot> robots;

    // [r][edge] = weight
    std::vector<std::unordered_map<uint32_t, uint32_t>> weights;

    [[nodiscard]] std::pair<double, std::array<std::vector<Action>, PIBT_LNS_DEPTH>> simulate(TimePoint end_time) const;

    void update_weights(const std::array<std::vector<Action>, PIBT_LNS_DEPTH>& actions);

public:
    PIBT_LNS(std::vector<Robot> robots);

    std::vector<Action> solve(TimePoint end_time);
};
