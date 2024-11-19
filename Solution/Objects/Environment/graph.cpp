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