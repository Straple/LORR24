#include "guidance_graph.hpp"

#include "assert.hpp"

uint16_t GuidanceGraph::get(uint32_t pos, uint32_t dir, uint32_t action) const {
    ASSERT(pos < graph.size(), "invalid pos");
    ASSERT(dir < 4, "invalid dir");
    ASSERT(action < 4, "invalid action");
    return graph[pos][dir][action];
}

void GuidanceGraph::set(uint32_t pos, uint32_t dir, uint32_t action, uint16_t weight) {
    ASSERT(pos < graph.size(), "invalid pos");
    ASSERT(dir < 4, "invalid dir");
    ASSERT(action < 4, "invalid action");
    graph[pos][dir][action] = weight;
}

std::istream &operator>>(std::istream &input, GuidanceGraph &gg) {
    uint32_t size;
    input >> size;
    gg.graph.resize(size);
    for (uint32_t pos = 0; pos < size; pos++) {
        for (uint32_t dir = 0; dir < 4; dir++) {
            for (uint32_t action = 0; action < 4; action++) {
                input >> gg.graph[pos][dir][action];
            }
        }
    }
    return input;
}

std::ostream &operator<<(std::ostream &output, const GuidanceGraph &gg) {
    output << gg.graph.size() << '\n';
    for (uint32_t pos = 0; pos < gg.graph.size(); pos++) {
        for (uint32_t dir = 0; dir < 4; dir++) {
            for (uint32_t action = 0; action < 4; action++) {
                output << gg.graph[pos][dir][action] << ' ';
            }
            output << '\n';
        }
        output << '\n';
    }
    return output;
}

GuidanceGraph &get_gg() {
    static GuidanceGraph gg;
    return gg;
}