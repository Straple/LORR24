#include <Objects/Environment/graph_guidance.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/position.hpp>
//#include <Objects/Environment/graph.hpp>
//#include <Objects/Environment/heuristic_matrix.hpp>

uint16_t PENALTY_WEIGHT = 6;
uint16_t OK_WEIGHT = 2;

void GraphGuidance::set(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t dir, uint32_t action, uint32_t value) {
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

void GraphGuidance::set_default() {
    for (uint32_t x = 0; x < rows; x++) {
        for (uint32_t y = 0; y < cols; y++) {
            uint32_t pos = x * cols + y + 1;
            for (uint32_t dir = 0; dir < 4; dir++) {
                for (uint32_t action = 0; action < 4; action++) {
                    graph[pos][dir][action] = OK_WEIGHT;
                }
            }
        }
    }
}

void GraphGuidance::set_grid() {
    for (uint32_t x = 0; x < rows; x++) {
        if (x & 1) {
            set(x, 0, x, cols - 1, 0, 0, PENALTY_WEIGHT);
            set(x, 0, x, cols - 1, 2, 0, OK_WEIGHT);
        } else {
            set(x, 0, x, cols - 1, 2, 0, PENALTY_WEIGHT);
            set(x, 0, x, cols - 1, 0, 0, OK_WEIGHT);
        }
    }
    for (uint32_t y = 0; y < cols; y++) {
        if (y & 1) {
            set(0, y, rows - 1, y, 1, 0, PENALTY_WEIGHT);
            set(0, y, rows - 1, y, 3, 0, OK_WEIGHT);
        } else {
            set(0, y, rows - 1, y, 3, 0, PENALTY_WEIGHT);
            set(0, y, rows - 1, y, 1, 0, OK_WEIGHT);
        }
    }
}

void GraphGuidance::set_warehouse() {
    set_grid();

    uint32_t msk = 0b10100101;
    int bit = 0;
    for (uint32_t y = 0; y < cols; y++) {
        if ((msk >> bit) & 1) {
            set(0, y, rows - 1, y, 1, 0, PENALTY_WEIGHT);
            set(0, y, rows - 1, y, 3, 0, OK_WEIGHT);
        } else {
            set(0, y, rows - 1, y, 3, 0, PENALTY_WEIGHT);
            set(0, y, rows - 1, y, 1, 0, OK_WEIGHT);
        }
        bit = (bit + 1) % 8;
    }
}

void GraphGuidance::set_sortation() {
    {
        uint32_t msk = 0b1001;
        int bit = 0;
        for (uint32_t x = 0; x < rows; x++) {
            if ((msk >> bit) & 1) {
                set(x, 0, x, cols - 1, 0, 0, PENALTY_WEIGHT);
                set(x, 0, x, cols - 1, 2, 0, OK_WEIGHT);
            } else {
                set(x, 0, x, cols - 1, 2, 0, PENALTY_WEIGHT);
                set(x, 0, x, cols - 1, 0, 0, OK_WEIGHT);
            }
            bit = (bit + 1) % 4;
        }
    }

    {
        uint32_t msk = 0b1100;
        int bit = 0;
        for (uint32_t y = 0; y < cols; y++) {
            if ((msk >> bit) & 1) {
                set(0, y, rows - 1, y, 1, 0, PENALTY_WEIGHT);
                set(0, y, rows - 1, y, 3, 0, OK_WEIGHT);
            } else {
                set(0, y, rows - 1, y, 3, 0, PENALTY_WEIGHT);
                set(0, y, rows - 1, y, 1, 0, OK_WEIGHT);
            }
            bit = (bit + 1) % 4;
        }
    }
}

void GraphGuidance::set_game() {
    set_grid();
}

void GraphGuidance::set_city() {
    set_grid();
}

GraphGuidance::GraphGuidance(uint32_t rows, uint32_t cols) : rows(rows), cols(cols), graph(rows * cols + 1) {
}

GraphGuidance::GraphGuidance(SharedEnvironment &env) : rows(env.rows), cols(env.cols), graph(env.rows * env.cols + 1) {
    if (get_map_type() == MapType::WAREHOUSE) {
        set_warehouse();
    } else if (get_map_type() == MapType::SORTATION) {
        set_sortation();
    } else if (get_map_type() == MapType::GAME) {
        set_game();
    } else if (get_map_type() == MapType::CITY) {
        set_city();
    } else if (get_map_type() == MapType::GAME) {
        set_default();
    } else {
        FAILED_ASSERT("undefined map");
    }

    for (uint32_t x = 0; x < rows; x++) {
        for (uint32_t y = 0; y < cols; y++) {
            uint32_t pos = x * cols + y + 1;
            for (uint32_t dir = 0; dir < 4; dir++) {
                for (uint32_t action = 0; action < 4; action++) {
                    if (!Position(x, y, dir).is_valid()) {
                        graph[pos][dir][action] = 0;
                    }
                }
            }
        }
    }

    // [pos][dir]
    /*get_graph() = Graph(get_map(), *this);
    get_hm() = HeuristicMatrix(get_graph());
    std::vector<std::vector<uint64_t>> kek(get_map().get_size(), std::vector<uint64_t>(4));
    uint64_t mx = 0;
    for (uint32_t x = 0; x < rows; x++) {
        for (uint32_t y = 0; y < cols; y++) {
            uint32_t pos = x * cols + y + 1;
            for (uint32_t dir = 0; dir < 4; dir++) {
                if (!Position(x, y, dir).is_valid()) {
                    continue;
                }
                uint32_t source = get_graph().get_node(Position(x, y, dir));
                uint64_t sum = 0;
                for (uint32_t target = 1; target < get_map().get_size(); target++) {
                    if (Position(target, 0).is_valid()) {
                        sum += get_hm().get(source, target);
                    }
                }
                kek[pos][dir] = sum;
                mx = std::max(mx, sum);
                uint32_t w = sum / get_map().get_size() + 1;
                graph[pos][dir][0] = w;
                graph[pos][dir][1] = w;
                graph[pos][dir][2] = w;
                graph[pos][dir][3] = w;
            }
        }
    }

    for (uint32_t x = 0; x < rows; x++) {
        for (uint32_t y = 0; y < cols; y++) {
            uint32_t pos = x * cols + y + 1;
            for (uint32_t dir = 0; dir < 4; dir++) {
                if (!Position(x, y, dir).is_valid()) {
                    continue;
                }

                for (uint32_t action = 0; action < 4; action++) {
                    graph[pos][dir][action] = mx * 10 / kek[pos][dir];
                }
            }
        }
    }*/

    /*{
        std::ofstream output("graph_guidance");
        output << *this;
    }
    _exit(0);*/
}

uint32_t GraphGuidance::get(uint32_t pos, uint32_t dir, uint32_t action) const {
    ASSERT(pos < graph.size(), "invalid pos");
    ASSERT(dir < 4, "invalid dir");
    ASSERT(action < 4, "invalid action");
    return graph[pos][dir][action];
}

void GraphGuidance::set(uint32_t pos, uint32_t dir, uint32_t action, uint32_t weight) {
    ASSERT(pos < graph.size(), "invalid pos");
    ASSERT(dir < 4, "invalid dir");
    ASSERT(action < 4, "invalid action");
    graph[pos][dir][action] = weight;
}

std::istream &operator>>(std::istream &input, GraphGuidance &gg) {
    uint32_t rows, cols;
    input >> rows >> cols;
    gg = GraphGuidance(rows, cols);
    for (uint32_t dir = 0; dir < 4; dir++) {
        for (uint32_t action = 0; action < 4; action++) {
            for (uint32_t pos = 1; pos < gg.graph.size(); pos++) {
                input >> gg.graph[pos][dir][action];
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

uint32_t GraphGuidance::get_size() const {
    return graph.size();
}

uint32_t GraphGuidance::get_rows() const {
    return rows;
}

uint32_t GraphGuidance::get_cols() const {
    return cols;
}

GraphGuidance &get_gg() {
    static GraphGuidance gg;
    return gg;
}
