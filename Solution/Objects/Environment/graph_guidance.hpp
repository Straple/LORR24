#pragma once

#include <array>
#include <cstdint>
#include <iostream>
#include <vector>

#include <Objects/Environment/map.hpp>

struct GraphGuidance {
    // weight[pos][dir][action]
    std::vector<std::array<std::array<uint16_t, 4>, 4>> graph;

public:
    GraphGuidance() = default;

    explicit GraphGuidance(const Map &map);

    [[nodiscard]] uint32_t get(uint32_t pos, uint32_t dir, uint32_t action) const;

    void set(uint32_t pos, uint32_t dir, uint32_t action, uint16_t weight);

    friend std::istream &operator>>(std::istream &input, GraphGuidance &gg);

    friend std::ostream &operator<<(std::ostream &output, const GraphGuidance &gg);
};

GraphGuidance &get_gg();
