#include <Objects/Environment/heuristic_matrix.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Containers/linear_heap.hpp>
#include <settings.hpp>

#include <thread>

void HeuristicMatrix::build(uint32_t source, const Graph &graph) {
    auto &dists = matrix[source];
    dists.assign(graph.get_nodes_size(), -1);

    // (dist, node)
    std::priority_queue<std::pair<uint32_t, uint32_t>> heap;

    std::vector<bool> visited(graph.get_nodes_size());

    for (uint32_t dir = 0; dir < 4; dir++) {
        heap.push({0, get_graph().get_node(Position(source, dir))});
    }

    while (!heap.empty()) {
        auto [dist, node] = heap.top();
        heap.pop();

        if (visited[node]) {
            continue;
        }
        visited[node] = true;

        uint32_t inv = get_graph().get_node(get_graph().get_pos(node).rotate().rotate());
        dists[inv] = dist;

        for (uint32_t action = 0; action < 3 /*WITHOUT WAIT = 3*/; action++) {
            uint32_t to = get_graph().get_to_node(node, action);
            if (to && !visited[to]) {
                uint32_t to_inv = get_graph().get_node(get_graph().get_pos(to).rotate().rotate());
                uint64_t to_dist = dist + 1;
                if (dists[to_inv] > to_dist) {
                    dists[to_inv] = to_dist;
                    heap.push({to_dist, to});
                }
            }
        }
    }
}

HeuristicMatrix::HeuristicMatrix(const Graph &graph) {
#ifdef ENABLE_HEURISTIC_MATRIX
    matrix.resize(get_map().get_size());

    auto do_work = [&](uint32_t thr) {
        for (uint32_t pos = thr + 1; pos < matrix.size(); pos += THREADS) {
            if(Position(pos, 0).is_valid()) {
                build(pos, graph);
            }
        }
    };

    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(do_work, thr);
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }
#endif
}

/*uint32_t HeuristicMatrix::get(uint32_t source, uint32_t dest) const {
    if (!dest) {
        return INVALID_DIST;
    }

#ifdef ENABLE_HEURISTIC_MATRIX
    ASSERT(0 < source && source < dp.size(), "invalid source");
    ASSERT(0 < dest && dest < dp.size(), "invalid dest");
    return dp[source][dest];
#else
    Position a = get_graph().get_pos(source);
    Position b = get_graph().get_pos(dest);
    return std::abs(static_cast<int32_t>(a.get_x()) - static_cast<int32_t>(b.get_x())) +
           std::abs(static_cast<int32_t>(a.get_y()) - static_cast<int32_t>(b.get_y()));
#endif
}*/

uint32_t HeuristicMatrix::get_to_pos(uint32_t source, uint32_t target) const {
    if (!target) {
        return INVALID_DIST;
    }
    ASSERT(0 < target && target < get_map().get_size(), "invalid dest");
    ASSERT(Position(target, 0).is_valid(), "invalid");
    ASSERT(Position(target, 1).is_valid(), "invalid");
    ASSERT(Position(target, 2).is_valid(), "invalid");
    ASSERT(Position(target, 3).is_valid(), "invalid");

    ASSERT(target < matrix.size(), "invalid target");
    ASSERT(source < matrix[target].size(), "invalid source");

    return matrix[target][source];

    /*uint32_t e = get_graph().get_node(Position(dest, 0));
    uint32_t s = get_graph().get_node(Position(dest, 1));
    uint32_t w = get_graph().get_node(Position(dest, 2));
    uint32_t n = get_graph().get_node(Position(dest, 3));
    return std::min(std::min(get(source, e), get(source, s)), std::min(get(source, w), get(source, n)));*/
}

HeuristicMatrix &get_hm() {
    static HeuristicMatrix hm;
    return hm;
}
