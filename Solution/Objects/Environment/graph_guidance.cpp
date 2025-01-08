#include <Objects/Environment/graph_guidance.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/position.hpp>

void GraphGuidance::set(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t dir, uint32_t action, uint16_t value) {
    for (int32_t x = x0; x <= x1; x++) {
        for (int32_t y = y0; y <= y1; y++) {
            int32_t pos = x * cols + y + 1;
            ASSERT(0 < pos && pos < graph.size(), "invalid pos");
            ASSERT(dir < 4, "invalid dir");
            ASSERT(action < 4, "invalid action");
            graph[pos][dir][action] = value;
        }
    }
}

void GraphGuidance::set_warehouse() {
    for (uint32_t x = 0; x < rows; x++) {
        for (uint32_t y = 0; y < cols; y++) {
            uint32_t pos = x * cols + y + 1;

            for (uint32_t dir = 0; dir < 4; dir++) {
                for (uint32_t action = 0; action < 4; action++) {
                    graph[pos][dir][action] = 2;

                    if (action != 0) {
                        continue;
                    }

                    if (y % 2 == 0) {
                        if (dir == 1) {
                            graph[pos][dir][action] = 3;
                        }
                    } else {
                        if (dir == 3) {
                            graph[pos][dir][action] = 3;
                        }
                    }

                    if (x % 2 == 0) {
                        if (dir == 0) {
                            graph[pos][dir][action] = 3;
                        }
                    } else {
                        if (dir == 2) {
                            graph[pos][dir][action] = 3;
                        }
                    }
                }
            }
        }
    }

    int kek = 0;
    for (uint32_t y = 3; y < cols; y += 4) {
        if (kek == 0) {
            kek = 1;
            set(0, y, rows - 1, y, 1, 0, 3);
        } else {
            kek = 0;
            set(0, y, rows - 1, y, 3, 0, 3);
        }
    }
}

void GraphGuidance::set_sortation() {
    for (uint32_t x = 0; x < rows; x++) {
        for (uint32_t y = 0; y < cols; y++) {
            uint32_t pos = x * cols + y + 1;

            for (uint32_t dir = 0; dir < 4; dir++) {
                for (uint32_t action = 0; action < 4; action++) {
                    graph[pos][dir][action] = 2;

                    if (action != 0) {
                        continue;
                    }

                    if (y % 2 == 0) {
                        if (dir == 1) {
                            graph[pos][dir][action] = 3;
                        }
                    } else {
                        if (dir == 3) {
                            graph[pos][dir][action] = 3;
                        }
                    }

                    if (x % 2 == 0) {
                        if (dir == 0) {
                            graph[pos][dir][action] = 3;
                        }
                    } else {
                        if (dir == 2) {
                            graph[pos][dir][action] = 3;
                        }
                    }
                }
            }
        }
    }

    int kek = 0;
    for (uint32_t x = 1; x < rows; x += 2) {
        if (kek == 0) {
            kek = 1;
            set(x, 0, x, cols - 1, 0, 0, 3);
        } else {
            kek = 0;
            set(x, 0, x, cols - 1, 2, 0, 3);
        }
    }
}

void GraphGuidance::set_game() {
    for (uint32_t x = 0; x < rows; x++) {
        for (uint32_t y = 0; y < cols; y++) {
            uint32_t pos = x * cols + y + 1;

            for (uint32_t dir = 0; dir < 4; dir++) {
                for (uint32_t action = 0; action < 4; action++) {
                    graph[pos][dir][action] = 2;

                    if (action != 0) {
                        continue;
                    }

                    // continue; // 2097

                    // 2136

                    if (y % 2 == 0) {
                        if (dir == 1) {
                            graph[pos][dir][action] = 3;
                        }
                    } else {
                        if (dir == 3) {
                            graph[pos][dir][action] = 3;
                        }
                    }

                    if (x % 2 == 0) {
                        if (dir == 0) {
                            graph[pos][dir][action] = 3;
                        }
                    } else {
                        if (dir == 2) {
                            graph[pos][dir][action] = 3;
                        }
                    }
                }
            }
        }
    }
}

GraphGuidance::GraphGuidance(SharedEnvironment &env, const Map &map) : rows(map.get_rows()), cols(map.get_cols()) {
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
    //call(0): 2459, 21.4121s
    //call(1): 4277, 45.2602s
    //call(2): 5237, 38.4895s
    //call(3): 5483, 56.0016s
    //call(4): 4652, 95.0316s
    //call(5): 3712, 168.167s
    //total: 25820

    graph.resize(map.get_size());
    if (env.map_name == "warehouse_large.map") {
        set_warehouse();
    } else if (env.map_name == "sortation_large.map") {
        set_sortation();
    } else if (env.map_name == "brc202d.map") {
        set_game();
    } else {
        for (uint32_t x = 0; x < rows; x++) {
            for (uint32_t y = 0; y < cols; y++) {
                uint32_t pos = x * cols + y + 1;
                for (uint32_t dir = 0; dir < 4; dir++) {
                    for (uint32_t action = 0; action < 4; action++) {
                        graph[pos][dir][action] = 2;
                    }
                }
            }
        }
    }
    /*{
        std::ofstream output("graph_guidance");
        output << *this;
    }
    std::exit(0);*/
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
    ASSERT(false, "outdated");
    for (uint32_t dir = 0; dir < 4; dir++) {
        for (uint32_t action = 0; action < 4; action++) {
            for (uint32_t x = 0; x < gg.rows; x++) {
                for (uint32_t y = 0; y < gg.cols; y++) {
                    uint32_t pos = x * gg.cols + y + 1;
                    input >> gg.graph[pos][dir][action];
                }
            }
        }
    }
    return input;
}

std::ostream &operator<<(std::ostream &output, const GraphGuidance &gg) {
    output << gg.rows << ' ' << gg.cols << '\n';
    for (uint32_t dir = 0; dir < 4; dir++) {
        for (uint32_t action = 0; action < 4; action++) {
            for (uint32_t pos = 1; pos < gg.graph.size(); pos++) {
                if (pos != 1) {
                    output << ' ';
                }
                output << gg.graph[pos][dir][action];
            }
            output << '\n';
        }
    }
    return output;
}

GraphGuidance &get_gg() {
    static GraphGuidance gg;
    return gg;
}
