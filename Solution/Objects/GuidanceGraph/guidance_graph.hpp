#pragma once

#include <array>
#include <cstdint>
#include <iostream>
#include <vector>

struct GuidanceGraph {
    // graph[pos][dir][action] = weight
    std::vector<std::array<std::array<uint16_t, 4>, 4>> graph;

public:
    [[nodiscard]] uint16_t get(uint32_t pos, uint32_t dir, uint32_t action) const;

    void set(uint32_t pos, uint32_t dir, uint32_t action, uint16_t weight);

    friend std::istream &operator>>(std::istream &input, GuidanceGraph &gg);

    friend std::ostream &operator<<(std::ostream &output, const GuidanceGraph &gg);
};

GuidanceGraph &get_gg();
