#include <Objects/Environment/heuristic_matrix.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Containers/linear_heap.hpp>

void HeuristicMatrix::build(uint32_t source, const Graph &graph) {
    dp[source].resize(graph.get_nodes_size());

    // (dist, node)
    LinearHeap<std::pair<uint32_t, uint32_t>> heap;

    std::vector<bool> visited(graph.get_nodes_size());

    heap.push({0, source});

    while (!heap.empty()) {
        auto [dist, node] = heap.top();
        heap.pop();

        if (visited[node]) {
            continue;
        }
        visited[node] = true;

        for (uint32_t action = 0; action < 3 /*WITHOUT WAIT = 3*/; action++) {
            uint32_t to = graph.get_to_node(node, action);
            if (to != 0 && !visited[to]) {
                heap.push({dist + 1, to});
            }
        }
    }
}

HeuristicMatrix::HeuristicMatrix(const Graph &graph) {
    dp.resize(graph.get_nodes_size());
    // TODO: multithread
    for (uint32_t node = 1; node < dp.size(); node++) {
        build(node, graph);
    }
}

uint32_t HeuristicMatrix::get(uint32_t source, uint32_t dest) const {
    ASSERT(0 < source && source < dp.size(), "invalid source");
    ASSERT(0 < dest && dest < dp.size(), "invalid dest");
    return dp[source][dest];
}