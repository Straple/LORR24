#pragma once

#include <Objects/Environment/graph.hpp>

class HeuristicMatrix {
    // dp[source][dest] = dist
    std::vector<std::vector<uint16_t>> dp;

    void build(uint32_t source, const Graph &graph);

public:
    HeuristicMatrix() = default;

    HeuristicMatrix(const Graph &graph);

    // source and dest is graph node
    // if dest is NONE(zero), then returns INVALID_DIST
    [[nodiscard]] uint32_t get(uint32_t source, uint32_t dest) const;

    // source is graph node
    // dest is map pos
    // if dest is NONE(zero), then returns INVALID_DIST
    [[nodiscard]] uint32_t get_to_pos(uint32_t source, uint32_t dest) const;
};

HeuristicMatrix &get_hm();
