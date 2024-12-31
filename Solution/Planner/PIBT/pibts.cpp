#include <Planner/PIBT/pibts.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>
#include <thread>


bool PIBTS::validate_path(uint32_t r, uint32_t desired) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < actions.size(), "invalid desired");

    uint32_t node = robots[r].node;
    auto &path = actions[desired];
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);

        if (!to_node || !to_edge) {
            return false;
        }

        node = to_node;
    }
    return true;
}

bool PIBTS::is_free_path(uint32_t r) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < actions.size(), "invalid desired");
    ASSERT(validate_path(r, desires[r]), "invalid path");

    uint32_t node = robots[r].node;
    auto &path = actions[desires[r]];
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        if (used_edge[depth][to_edge] != -1) {
            ASSERT(used_edge[depth][to_edge] != r, "invalid used_edge");
            return false;
        }

        if (used_pos[depth][to_pos] != -1) {
            ASSERT(used_pos[depth][to_pos] != r, "invalid used_node");
            return false;
        }

        node = to_node;
    }
    return true;
}

bool PIBTS::is_free_path(uint32_t r, const State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < actions.size(), "invalid desired");
    ASSERT(validate_path(r, desires[r]), "invalid path");

    uint32_t node = robots[r].node;
    auto &path = actions[desired];
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        uint32_t used_edge_value = state.used_edge[depth].count(to_edge) ? state.used_edge[depth].at(to_edge)
                                                                         : used_edge[depth][to_edge];

        uint32_t used_pos_value = state.used_pos[depth].count(to_pos) ? state.used_pos[depth].at(to_pos)
                                                                      : used_pos[depth][to_pos];

        if (used_edge_value != -1) {
            ASSERT(used_edge_value != r, "invalid used_edge");
            return false;
        }

        if (used_pos_value != -1) {
            ASSERT(used_pos_value != r, "invalid used_node");
            return false;
        }

        node = to_node;
    }
    return true;
}

std::array<uint32_t, DEPTH> PIBTS::get_path(uint32_t r, uint32_t desired) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < actions.size(), "invalid desired");
    ASSERT(validate_path(r, desired), "invalid path");

    uint32_t node = robots[r].node;
    auto &path = actions[desired];
    std::array<uint32_t, DEPTH> result{};
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);

        result[depth] = to_node;
        node = to_node;
    }
    return result;
}

uint32_t PIBTS::get_used(uint32_t r) const {
    uint32_t node = robots[r].node;
    auto &path = actions[desires[r]];
    uint32_t answer = -1;
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        if (used_edge[depth][to_edge] != -1) {
            if (answer == -1) {
                answer = used_edge[depth][to_edge];
            } else if (answer != used_edge[depth][to_edge]) {
                answer = -2;
                break;
            }
        }

        if (used_pos[depth][to_pos] != -1) {
            if (answer == -1) {
                answer = used_pos[depth][to_pos];
            } else if (answer != used_pos[depth][to_pos]) {
                answer = -2;
                break;
            }
        }
        node = to_node;
    }
    return answer;
}

uint32_t PIBTS::get_used(uint32_t r, const State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    uint32_t node = robots[r].node;
    auto &path = actions[desired];
    uint32_t answer = -1;
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        uint32_t used_edge_value = state.used_edge[depth].count(to_edge) ? state.used_edge[depth].at(to_edge)
                                                                         : used_edge[depth][to_edge];

        uint32_t used_pos_value = state.used_pos[depth].count(to_pos) ? state.used_pos[depth].at(to_pos)
                                                                      : used_pos[depth][to_pos];

        if (used_edge_value != -1) {
            if (answer == -1) {
                answer = used_edge_value;
            } else if (answer != used_edge_value) {
                answer = -2;
                break;
            }
        }

        if (used_pos_value != -1) {
            if (answer == -1) {
                answer = used_pos_value;
            } else if (answer != used_pos_value) {
                answer = -2;
                break;
            }
        }

        node = to_node;
    }
    return answer;
}

void PIBTS::update_score(uint32_t r, uint32_t finish_node, double &cur_score, int sign) const {
    int32_t old_dist = get_hm().get_to_pos(robots[r].node, robots[r].target);
    int32_t cur_dist = get_hm().get_to_pos(finish_node, robots[r].target);
    int32_t diff = old_dist - cur_dist;
    double power = (order[r] + 1) * 1.0 / robots.size();
    cur_score += sign * diff * power;
}

void PIBTS::add_path(uint32_t r) {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < actions.size(), "invalid desired: " + std::to_string(desires[r]) + " " + std::to_string(actions.size()));

    uint32_t node = robots[r].node;
    ASSERT(node != 0, "invalid start node");
    auto &path = actions[desires[r]];
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        ASSERT(to_node, "invalid to_node");
        ASSERT(to_pos < used_pos[depth].size(), "invalid to_pos");
        ASSERT(to_edge && to_edge < used_edge[depth].size(), "invalid to_edge");

        ASSERT(used_edge[depth][to_edge] == -1, "already used");
        used_edge[depth][to_edge] = r;

        ASSERT(used_pos[depth][to_pos] == -1, "already used");
        used_pos[depth][to_pos] = r;

        node = to_node;
    }

    update_score(r, node, cur_score, +1);
}

void PIBTS::add_path(uint32_t r, State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < actions.size(), "invalid desired: " + std::to_string(desired) + " " + std::to_string(actions.size()));

    uint32_t node = robots[r].node;
    ASSERT(node != 0, "invalid start node");
    auto &path = actions[desired];
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        ASSERT(to_node, "invalid to_node");
        ASSERT(to_pos < used_pos[depth].size(), "invalid to_pos");
        ASSERT(to_edge && to_edge < used_edge[depth].size(), "invalid to_edge");

        uint32_t used_edge_value = state.used_edge[depth].count(to_edge) ? state.used_edge[depth].at(to_edge)
                                                                         : used_edge[depth][to_edge];

        uint32_t used_pos_value = state.used_pos[depth].count(to_pos) ? state.used_pos[depth].at(to_pos)
                                                                      : used_pos[depth][to_pos];

        ASSERT(used_edge_value == -1, "already used");
        state.used_edge[depth][to_edge] = r;

        ASSERT(used_pos_value == -1, "already used");
        state.used_pos[depth][to_pos] = r;

        node = to_node;
    }

    update_score(r, node, state.cur_score, +1);
}

void PIBTS::remove_path(uint32_t r) {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < actions.size(), "invalid desired");

    uint32_t node = robots[r].node;
    auto &path = actions[desires[r]];
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        ASSERT(to_node, "invalid to_node");
        ASSERT(to_pos < used_pos[depth].size(), "invalid to_pos");
        ASSERT(to_edge && to_edge < used_edge[depth].size(), "invalid to_edge");

        ASSERT(used_edge[depth][to_edge] == r, "invalid edge");
        used_edge[depth][to_edge] = -1;

        ASSERT(used_pos[depth][to_pos] == r, "invalid node");
        used_pos[depth][to_pos] = -1;

        node = to_node;
    }

    update_score(r, node, cur_score, -1);
}

void PIBTS::remove_path(uint32_t r, State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < actions.size(), "invalid desired");

    uint32_t node = robots[r].node;
    auto &path = actions[desired];
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        ASSERT(to_node, "invalid to_node");
        ASSERT(to_pos < used_pos[depth].size(), "invalid to_pos");
        ASSERT(to_edge && to_edge < used_edge[depth].size(), "invalid to_edge");

        uint32_t used_edge_value = state.used_edge[depth].count(to_edge) ? state.used_edge[depth].at(to_edge)
                                                                         : used_edge[depth][to_edge];

        uint32_t used_pos_value = state.used_pos[depth].count(to_pos) ? state.used_pos[depth].at(to_pos)
                                                                      : used_pos[depth][to_pos];

        ASSERT(used_edge_value == r, "invalid edge");
        state.used_edge[depth][to_edge] = -1;

        ASSERT(used_pos_value == r, "invalid node");
        state.used_pos[depth][to_pos] = -1;

        node = to_node;
    }

    update_score(r, node, state.cur_score, -1);
}

void PIBTS::flush_state(const State &state) {
    cur_score = state.cur_score;
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        for (auto [pos, value]: state.used_pos[depth]) {
            used_pos[depth][pos] = value;
        }
        for (auto [edge, value]: state.used_edge[depth]) {
            used_edge[depth][edge] = value;
        }
    }
    for (auto [r, desired]: state.desires) {
        desires[r] = desired;
    }
}

bool PIBTS::try_build(uint32_t r, State &state, uint32_t &counter, Randomizer &rnd) const {
    // (priority, desired)
    std::vector<std::pair<int64_t, uint32_t>> steps;
    for (uint32_t desired = 1; desired < actions.size(); desired++) {
        state.desires[r] = desired;
        if (!validate_path(r, desired) || get_used(r, state) == -2) {
            continue;
        }

        auto path = get_path(r, desired);

        int64_t priority = get_hm().get_to_pos(path.back(), robots[r].target);

        steps.emplace_back(priority, desired);
    }

    std::stable_sort(steps.begin(), steps.end());

    for (auto [_, desired]: steps) {
        state.desires[r] = desired;

        if(rnd.get_d() < 0.2){
            continue;
        }

        if (is_free_path(r, state)) {
            // отлично! там никого нет
            add_path(r, state);
            return true;
        } else {
            // о нет! там кто-то есть

            if (counter > 100) {
                continue;
            }

            uint32_t to_r = get_used(r, state);
            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
            if (state.desires.count(to_r)) {
                continue;
            }
            remove_path(to_r, state);
            add_path(r, state);

            if (try_build(to_r, state, ++counter, rnd)) {
                return true;
            }

            remove_path(r, state);
            add_path(to_r, state);
        }
    }

    state.desires.erase(r);
    return false;
}

bool PIBTS::try_build(uint32_t r, Randomizer &rnd) {
    State state;
    state.cur_score = cur_score;
    remove_path(r, state);
    uint32_t counter = 0;
    if (!try_build(r, state, counter, rnd)) {
        return false;
    }

    // TODO: annealing
    if (cur_score <= state.cur_score) {
        flush_state(state);
        return true;
    }
    return false;
}

PIBTS::PIBTS(const std::vector<Robot> &robots) : robots(robots) {

    desires.resize(robots.size());

    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        used_pos[depth].resize(get_map().get_size(), -1);
        used_edge[depth].resize(get_graph().get_edges_size(), -1);
    }

    // build order
    {
        std::vector<uint32_t> p(robots.size());
        iota(p.begin(), p.end(), 0);
        std::stable_sort(p.begin(), p.end(), [&](uint32_t lhs, uint32_t rhs) {
            return robots[lhs].priority > robots[rhs].priority;
        });

        order.resize(robots.size());
        for (uint32_t i = 0; i < robots.size(); i++) {
            order[p[i]] = i;
        }
    }

    for (uint32_t r = 0; r < this->robots.size(); r++) {
        add_path(r);
    }
}

std::vector<Action> PIBTS::solve(TimePoint end_time) {
    this->end_time = end_time;
    Timer timer;
    static Randomizer rnd;
    //std::vector<std::thread> threads(THREADS);
    uint32_t cnt = 0;
    while (get_now() < end_time) {
        uint32_t r = rnd.get(0, robots.size() - 1);
        try_build(r, rnd);
        cnt++;
    }

    std::vector<Action> answer(robots.size());

    for (uint32_t r = 0; r < robots.size(); r++) {
        answer[r] = actions[desires[r]][0];
    }

    std::cout << "PIBTS: " << timer << ", " << cnt << '\n';
    return answer;
}
