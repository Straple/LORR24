#pragma once

#include <Objects/Basic/position.hpp>
#include <Objects/Environment/map.hpp>

#include <array>
#include <cstdint>
#include <vector>

// Contains information about graph.
// About edges, transform position to node and back
class Graph {

    // 0 = NONE

    // to[node][action] = to vertex
    std::vector<std::array<uint32_t, 4>> to;

    // pos_to_vertex[pos][dir] = graph vertex
    std::vector<std::array<uint32_t, 4>> pos_to_node;

    // [vertex] = position
    std::vector<Position> node_to_pos;

public:
    explicit Graph(const Map &map);

    [[nodiscard]] Position get_pos(uint32_t node) const;

    [[nodiscard]] uint32_t get_node(const Position &pos) const;
};
