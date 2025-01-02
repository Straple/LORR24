#pragma once

#include <Objects/Basic/time.hpp>
#include <Objects/Environment/map.hpp>

#include <SharedEnv.h>

#include <unordered_set>
#include <vector>

class DynamicHeuristicMatrix {
    // matrix[target (map pos)][source (graph node)] = dist source -> target
    std::vector<std::vector<uint32_t>> matrix;

    // timestep_updated[pos]
    std::vector<uint32_t> timestep_updated;

    void rebuild(uint32_t source, const std::unordered_set<uint32_t> &robots_pos);

public:
    DynamicHeuristicMatrix() = default;

    explicit DynamicHeuristicMatrix(const Map &map);

    void update(SharedEnvironment &env, TimePoint end_time);

    // source graph node -> target map pos
    [[nodiscard]] uint32_t get(uint32_t source, uint32_t target) const;
};

DynamicHeuristicMatrix &get_dhm();
