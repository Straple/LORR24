#include <Objects/Environment/operations_map.hpp>

#include <Objects/Basic/assert.hpp>

std::pair<EPath, EPath> OperationsMap::get_paths(uint32_t node, const Operation &operation) {
    EPath nodes_path{};
    EPath edges_path{};

    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = operation[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);

        if (!to_node || !to_edge) {
            return {EPath{}, EPath{}};
        }

        nodes_path[depth] = to_node;
        edges_path[depth] = to_edge;

        node = to_node;
    }
    return {nodes_path, edges_path};
}

void OperationsMap::build(uint32_t source, const std::vector<Operation> &operations) {
    auto &nodes = map_nodes[source];
    auto &edges = map_edges[source];

    for (auto operation: operations) {
        auto [nodes_path, edges_path] = get_paths(source, operation);
        nodes.push_back(nodes_path);
        edges.push_back(edges_path);
    }
}

OperationsMap::OperationsMap(const Graph &graph, const std::vector<Operation> &operations) {
    map_nodes.resize(graph.get_nodes_size());
    map_edges.resize(graph.get_nodes_size());

    // TODO: multithreading
    for (uint32_t node = 0; node < graph.get_nodes_size(); node++) {
        build(node, operations);
    }
}

const EPath &OperationsMap::get_nodes_path(uint32_t node, uint32_t operation) {
    ASSERT(node < map_nodes.size(), "invalid node");
    ASSERT(operation < map_nodes[node].size(), "invalid operation");
    return map_nodes[node][operation];
}

const EPath &OperationsMap::get_edges_path(uint32_t node, uint32_t operation) {
    ASSERT(node < map_edges.size(), "invalid node");
    ASSERT(operation < map_edges[node].size(), "invalid operation");
    return map_edges[node][operation];
}

OperationsMap &get_omap() {
    static OperationsMap omap;
    return omap;
}
