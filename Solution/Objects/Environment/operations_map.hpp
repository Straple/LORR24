#pragma once

#include <Objects/Environment/graph.hpp>
#include <Objects/Environment/operations.hpp>

class OperationsMap {

    // map_nodes[node][operation] = nodes path
    std::vector<std::vector<EPath>> map_nodes;

    // map_edges[node][operation] = edges path
    std::vector<std::vector<EPath>> map_edges;

    static std::pair<EPath, EPath> get_paths(uint32_t node, const Operation &operation);

    void build(uint32_t source, const std::vector<Operation> &operations);

public:
    OperationsMap() = default;

    OperationsMap(const Graph &graph, const std::vector<Operation> &operations);

    const EPath &get_nodes_path(uint32_t node, uint32_t operation);

    const EPath &get_edges_path(uint32_t node, uint32_t operation);
};

OperationsMap& get_omap();
