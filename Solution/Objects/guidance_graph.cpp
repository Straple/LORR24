#include "guidance_graph.hpp"

#include "assert.hpp"

uint16_t GuidanceGraph::get(uint32_t pos, uint32_t dir) const {
    ASSERT(pos < graph.size(), "invalid pos");
    ASSERT(dir < 4, "invalid dir");
    return graph[pos][dir];
}

void GuidanceGraph::set(uint32_t pos, uint32_t dir, uint16_t weight) {
    ASSERT(pos < graph.size(), "invalid pos");
    ASSERT(dir < 4, "invalid dir");
    graph[pos][dir] = weight;
}

std::istream &operator>>(std::istream &input, GuidanceGraph &gg) {
    uint32_t size;
    input >> size;
    gg.graph.resize(size);
    for (uint32_t pos = 0; pos < size; pos++) {
        for (uint32_t dir = 0; dir < 4; dir++) {
            input >> gg.graph[pos][dir];
        }
    }
    return input;
}
