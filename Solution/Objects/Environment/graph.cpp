#include <Objects/Environment/graph.hpp>

#include <Objects/Basic/assert.hpp>

Graph::Graph(const Map &map) {
    pos_to_node.resize(map.get_size());
    node_to_pos.resize(1);
    for (uint32_t pos = 0; pos < map.get_size(); pos++) {
        if (!map.is_free(pos)) {
            continue;
        }
        for (uint32_t dir = 0; dir < 4; dir++) {
            Position p(pos, dir);
            ASSERT(p.is_valid(), "p is invalid");
            pos_to_node[pos][dir] = node_to_pos.size();
            node_to_pos.push_back(p);
        }
    }

    to_node.resize(node_to_pos.size());
    to_edge.resize(node_to_pos.size());

    std::map<std::pair<uint32_t, uint32_t>, uint32_t> edges;
    for (uint32_t node = 0; node < node_to_pos.size(); node++) {
        for (uint32_t action = 0; action < 4; action++) {
            Position p = node_to_pos[node];
            Position to = p.simulate_action(static_cast<Action>(action));
            if (!to.is_valid()) {
                continue;
            }

            to_node[node][action] = get_node(to);

            uint32_t a = p.get_pos();
            uint32_t b = to.get_pos();

            if (a > b) {
                std::swap(a, b);
            }
            if (!edges.count({a, b})) {
                edges[{a, b}] = edges.size() + 1;
            }

            to_edge[node][action] = edges[{a, b}];
        }
    }
    edges_size = edges.size() + 1;
}

uint32_t Graph::get_nodes_size() const {
    return node_to_pos.size();
}

uint32_t Graph::get_edges_size() const {
    return edges_size;
}

Position Graph::get_pos(uint32_t node) const {
    ASSERT(0 < node && node < node_to_pos.size(), "invalid node");
    return node_to_pos[node];
}

uint32_t Graph::get_node(const Position &pos) const {
    ASSERT(pos.is_valid(), "invalid position");
    ASSERT(0 <= pos.get_pos() && pos.get_pos() < pos_to_node.size(), "invalid pos");
    ASSERT(0 <= pos.get_dir() && pos.get_dir() < 4, "invalid dir");
    return pos_to_node[pos.get_pos()][pos.get_dir()];
}

uint32_t Graph::get_to_node(uint32_t node, uint32_t action) const {
    ASSERT(0 < node && node < to_node.size(), "invalid node");
    ASSERT(action < 4, "invalid action");
    return to_node[node][action];
}

uint32_t Graph::get_to_edge(uint32_t node, uint32_t action) const {
    ASSERT(0 < node && node < to_edge.size(), "invalid node");
    ASSERT(action < 4, "invalid action");
    return to_edge[node][action];
}

Graph &get_graph() {
    static Graph graph;
    return graph;
}
