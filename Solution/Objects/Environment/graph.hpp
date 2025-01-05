#pragma once

#include <Objects/Basic/position.hpp>
#include <Objects/Environment/map.hpp>

#include <array>
#include <cstdint>
#include <vector>

// Contains information about graph.
// About edges, transform position to node and back.
// Undirected edges.
// 0 = NONE
class Graph {

    // pos_to_node[pos][dir] = graph node
    std::vector<std::array<uint32_t, 4>> pos_to_node;

    // node_to_pos[node] = position
    std::vector<Position> node_to_pos;

    // to_node[node][action] = to node
    std::vector<std::array<uint32_t, 4>> to_node;

    // to_edge[node][action] = to edge
    std::vector<std::array<uint32_t, 4>> to_edge;

    uint32_t edges_size = 0;

public:
    Graph() = default;

    explicit Graph(const Map &map);

    [[nodiscard]] uint32_t get_nodes_size() const;

    [[nodiscard]] uint32_t get_edges_size() const;

    // graph node -> Position
    [[nodiscard]] Position get_pos(uint32_t node) const;

    // Position -> graph node
    [[nodiscard]] uint32_t get_node(const Position &pos) const;

    [[nodiscard]] uint32_t get_node(const uint32_t pos, const uint32_t dir) const;


    // graph node + action -> graph node
    [[nodiscard]] uint32_t get_to_node(uint32_t node, uint32_t action) const;

    // graph node + action -> graph edge
    [[nodiscard]] uint32_t get_to_edge(uint32_t node, uint32_t action) const;
};

Graph &get_graph();
