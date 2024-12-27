#pragma once

#include <Objects/Environment/graph.hpp>

class HeuristicMatrix {
    // dp[source][dest] = dist
    std::vector<std::vector<uint16_t>> dp;

    void build(uint32_t source, const Graph &graph);

public:
    HeuristicMatrix() = default;

    void init(const Graph &graph);

    [[nodiscard]] uint32_t get(uint32_t source, uint32_t dest) const;

    [[nodiscard]] uint32_t get_to_pos(uint32_t source, uint32_t dest) const;
};

HeuristicMatrix &get_hm();
