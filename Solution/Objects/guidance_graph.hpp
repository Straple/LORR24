#pragma once

#include <array>
#include <cstdint>
#include <iostream>
#include <vector>

class GuidanceGraph {
    // graph[pos][dir] = weight
    std::vector<std::array<uint16_t, 4>> graph;

public:
    [[nodiscard]] uint16_t get(uint32_t pos, uint32_t dir) const;

    void set(uint32_t pos, uint32_t dir, uint16_t weight);

    friend std::istream &operator>>(std::istream &input, GuidanceGraph &gg);
};
