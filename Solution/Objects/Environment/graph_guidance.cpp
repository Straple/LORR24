#include <Objects/Environment/graph_guidance.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/position.hpp>

GraphGuidance::GraphGuidance(const Map &map) {
    // without GG + without DHM
    //call(0): 2476, 13.3796s
    //call(1): 4067, 24.655s
    //call(2): 3819, 79.7985s
    //call(3): 3456, 208.104s
    //call(4): 3112, 380.675s
    //call(5): 2736, 627.792s
    //total: 19666

    // without GG + DHM
    //call(0): 2545, 17.2474s
    //call(1): 4398, 23.8929s
    //call(2): 5337, 39.4726s
    //call(3): 5574, 60.5013s
    //call(4): 4427, 108.857s
    //call(5): 3509, 166.195s
    //total: 25790

    graph.resize(map.get_size());
    for (uint32_t pos = 1; pos < graph.size(); pos++) {
        for (uint32_t dir = 0; dir < 4; dir++) {
            for (uint32_t action = 0; action < 4; action++) {
                graph[pos][dir][action] = 1;

                /*Position p(pos, dir);
                if (p.get_x() % 2 == 0) {
                    if (dir == 0) {
                        // east ->
                        graph[pos][dir][action] = 2;
                    }
                } else {
                    if (dir == 2) {
                        // west <-
                        graph[pos][dir][action] = 2;
                    }
                }*/
            }
        }
    }
}

uint32_t GraphGuidance::get(uint32_t pos, uint32_t dir, uint32_t action) const {
    ASSERT(pos < graph.size(), "invalid pos");
    ASSERT(dir < 4, "invalid dir");
    ASSERT(action < 4, "invalid action");
    return graph[pos][dir][action];
}

void GraphGuidance::set(uint32_t pos, uint32_t dir, uint32_t action, uint16_t weight) {
    ASSERT(pos < graph.size(), "invalid pos");
    ASSERT(dir < 4, "invalid dir");
    ASSERT(action < 4, "invalid action");
    graph[pos][dir][action] = weight;
}

std::istream &operator>>(std::istream &input, GraphGuidance &gg) {
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

std::ostream &operator<<(std::ostream &output, const GraphGuidance &gg) {
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

GraphGuidance &get_gg() {
    static GraphGuidance gg;
    return gg;
}
