#include <Planner/PIBT/pibts.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>

bool PIBTS::validate_path(uint32_t r, uint32_t desired) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");
    uint32_t node = robots[r].node;
    return get_omap().get_poses_path(node, desires[r])[0] > 0;
}

bool PIBTS::is_free_path(uint32_t r) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < get_operations().size(), "invalid desired");
    ASSERT(validate_path(r, desires[r]), "invalid path");

    uint32_t source = robots[r].node;
    auto &poses_path = get_omap().get_poses_path(source, desires[r]);
    auto &edges_path = get_omap().get_edges_path(source, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        if (used_pos[depth][poses_path[depth]] != -1) {
            ASSERT(used_pos[depth][poses_path[depth]] != r, "invalid used_node");
            return false;
        }
        if (used_edge[depth][edges_path[depth]] != -1) {
            ASSERT(used_edge[depth][edges_path[depth]] != r, "invalid used_edge");
            return false;
        }
    }
    return true;
}

bool PIBTS::is_free_path(uint32_t r, const State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");
    ASSERT(validate_path(r, desires[r]), "invalid path");

    uint32_t node = robots[r].node;
    auto &path = get_operations()[desired];
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

EPath PIBTS::get_path(uint32_t r, uint32_t desired) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");
    ASSERT(validate_path(r, desired), "invalid path");

    uint32_t node = robots[r].node;
    return get_omap().get_nodes_path(node, desired);
}

uint32_t PIBTS::get_used(uint32_t r) const {
    uint32_t node = robots[r].node;
    uint32_t answer = -1;

    uint32_t source = robots[r].node;
    auto &poses_path = get_omap().get_poses_path(source, desires[r]);
    auto &edges_path = get_omap().get_edges_path(source, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        uint32_t to_pos = poses_path[depth];

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
    }
    return answer;
}

uint32_t PIBTS::get_used(uint32_t r, const State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    uint32_t node = robots[r].node;
    auto &path = get_operations()[desired];
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
    int64_t old_dist = get_dhm().get(robots[r].node, robots[r].target);
    int64_t cur_dist = get_dhm().get(finish_node, robots[r].target);
    int64_t diff = (old_dist - cur_dist);// * (old_dist - cur_dist) * (old_dist - cur_dist);
    double power = (static_cast<int32_t>(robots.size()) - weight[r]) * 1.0 / robots.size();
    cur_score += sign * diff * power;
}

void PIBTS::add_path(uint32_t r) {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < get_operations().size(), "invalid desired: " + std::to_string(desires[r]) + " " + std::to_string(get_operations().size()));

    uint32_t source = robots[r].node;
    auto &poses_path = get_omap().get_poses_path(source, desires[r]);
    auto &edges_path = get_omap().get_edges_path(source, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        uint32_t to_pos = poses_path[depth];

        ASSERT(to_pos < used_pos[depth].size(), "invalid to_pos");
        ASSERT(to_edge && to_edge < used_edge[depth].size(), "invalid to_edge");

        ASSERT(used_edge[depth][to_edge] == -1, "already used");
        used_edge[depth][to_edge] = r;

        ASSERT(used_pos[depth][to_pos] == -1, "already used");
        used_pos[depth][to_pos] = r;
    }

    update_score(r, get_omap().get_nodes_path(source, desires[r]).back(), cur_score, +1);
}

void PIBTS::add_path(uint32_t r, State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired: " + std::to_string(desired) + " " + std::to_string(get_operations().size()));

    uint32_t node = robots[r].node;
    ASSERT(node != 0, "invalid start node");
    auto &path = get_operations()[desired];
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
    ASSERT(0 <= desires[r] && desires[r] < get_operations().size(), "invalid desired");

    uint32_t source = robots[r].node;
    auto &poses_path = get_omap().get_poses_path(source, desires[r]);
    auto &edges_path = get_omap().get_edges_path(source, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        uint32_t to_pos = poses_path[depth];

        ASSERT(to_pos < used_pos[depth].size(), "invalid to_pos");
        ASSERT(to_edge && to_edge < used_edge[depth].size(), "invalid to_edge");

        ASSERT(used_edge[depth][to_edge] == r, "invalid edge");
        used_edge[depth][to_edge] = -1;

        ASSERT(used_pos[depth][to_pos] == r, "invalid node");
        used_pos[depth][to_pos] = -1;
    }

    update_score(r, get_omap().get_nodes_path(source, desires[r]).back(), cur_score, -1);
}

void PIBTS::remove_path(uint32_t r, State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");

    uint32_t node = robots[r].node;
    auto &path = get_operations()[desired];
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
    for (uint32_t desired = 1; desired < get_operations().size(); desired++) {
        state.desires[r] = desired;
        if (!validate_path(r, desired) || get_used(r, state) == -2) {
            continue;
        }

        auto path = get_path(r, desired);

        int64_t priority = get_dhm().get(path.back(), robots[r].target);

        steps.emplace_back(priority, desired);
    }

    std::stable_sort(steps.begin(), steps.end());

    for (auto [_, desired]: steps) {
        state.desires[r] = desired;

        if (rnd.get_d() < 0.2) {
            continue;
        }

        if (is_free_path(r, state)) {
            // отлично! там никого нет
            add_path(r, state);
            return true;
        } else {
            // о нет! там кто-то есть

            if (counter > 1'000) {
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

uint32_t PIBTS::try_build2(uint32_t r, uint32_t &counter, Randomizer &rnd, double old_score) {
    visited[r] = visited_counter;

    // (priority, desired)
    std::vector<std::pair<int64_t, uint32_t>> steps;
    uint32_t old_desired = desires[r];
    for (uint32_t desired = 1; desired < get_operations().size(); desired++) {
        desires[r] = desired;
        if (!validate_path(r, desired)) {
            continue;
        }
        uint32_t to_r = get_used(r);
        if (to_r == -2) {
            continue;
        }
        const auto &path = get_path(r, desired);
        int64_t priority = get_dhm().get(path.back(), robots[r].target);
        steps.emplace_back(priority, desired);
    }

    std::stable_sort(steps.begin(), steps.end());

    for (auto [_, desired]: steps) {
        desires[r] = desired;

        //if (rnd.get_d() < 0.1) {
        //    continue;
        //}

        if (is_free_path(r)) {
            // отлично! там никого нет
            add_path(r);
            if (old_score <= cur_score
                // old_score > cur_score
                // || rnd.get_d() < 1.0 / (old_score - cur_score + 10) * temp
            ) {
                return 1;// accepted
            } else {
                remove_path(r);
                desires[r] = old_desired;
                return 2;// not accepted
            }
        } else {
            // о нет! там кто-то есть

            if (counter > 1'000) {
                continue;
            }

            uint32_t to_r = get_used(r);
            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
            if (visited[to_r] == visited_counter) {
                continue;
            }
            remove_path(to_r);
            add_path(r);

            uint32_t res = try_build2(to_r, ++counter, rnd, old_score);
            if (res == 1) {
                return res;
            } else if (res == 2) {
                remove_path(r);
                add_path(to_r);
                desires[r] = old_desired;
                return res;
            }

            remove_path(r);
            add_path(to_r);
        }
    }

    desires[r] = old_desired;
    visited[r] = 0;
    return 0;
}

bool PIBTS::try_build2(uint32_t r, Randomizer &rnd) {
    double old_score = cur_score;
    remove_path(r);
    uint32_t counter = 0;
    uint32_t res = try_build2(r, counter, rnd, old_score);
    if (res == 0 || res == 2) {
        add_path(r);
        return false;
    }
    return true;
}

bool PIBTS::build(uint32_t r, uint32_t depth, uint32_t &counter) {
    if (counter == -1 || (counter % 256 == 0 && get_now() >= end_time)) {
        counter = -1;
        return false;
    }

    uint32_t old_desired = desires[r];

    // (priority, desired)
    std::vector<std::pair<int64_t, uint32_t>> steps;
    for (uint32_t desired = 1; desired < get_operations().size(); desired++) {
        desires[r] = desired;
        if (!validate_path(r, desires[r])) {
            continue;
        }
        uint32_t to_r = get_used(r);
        if (to_r == -2) {
            continue;
        }
        if (to_r != -1 && desires[to_r] != 0) {
            continue;
        }
        const auto &path = get_path(r, desires[r]);

        int64_t priority = get_dhm().get(path.back(), robots[r].target);
        steps.emplace_back(priority, desired);
    }

    std::stable_sort(steps.begin(), steps.end());
    //std::reverse(steps.begin(), steps.end());

    for (auto [priority, desired]: steps) {
        desires[r] = desired;
        if (is_free_path(r)) {
            // отлично! там никого нет
            add_path(r);
            return true;
        } else {
            // о нет! там кто-то есть

            if (counter > 3'000 && depth > 6) {
                continue;
            } else if (counter > 10'000 && depth > 3) {
                continue;
            } else if (counter > 30'000) {
                continue;
            }

            uint32_t to_r = get_used(r);
            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
            ASSERT(desires[to_r] == 0, "invalid desires");
            remove_path(to_r);
            add_path(r);

            if (build(to_r, depth + 1, ++counter)) {
                return true;
            }

            remove_path(r);
            add_path(to_r);
        }
    }

    desires[r] = old_desired;
    return false;
}

bool PIBTS::build(uint32_t r) {
    remove_path(r);
    uint32_t counter = 0;
    if (!build(r, 0, counter)) {
        add_path(r);
        return false;
    } else {
        return true;
    }
}

PIBTS::PIBTS(const std::vector<Robot> &robots) : robots(robots) {

    visited.resize(robots.size());
    desires.resize(robots.size());

    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        used_pos[depth].resize(get_map().get_size(), -1);
        used_edge[depth].resize(get_graph().get_edges_size(), -1);
    }

    // build order
    {
        order.resize(robots.size());
        iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
            return robots[lhs].priority < robots[rhs].priority;
        });

        weight.resize(robots.size());
        for (uint32_t i = 0; i < robots.size(); i++) {
            weight[order[i]] = i;
        }
    }

    for (uint32_t r = 0; r < this->robots.size(); r++) {
        add_path(r);
    }
}

std::vector<Action> PIBTS::solve(TimePoint end_time, uint64_t seed) {
    this->end_time = end_time;
    Randomizer rnd(seed);

    //auto p = order;
    //std::shuffle(p.begin(), p.end(), rnd.generator);
    for (uint32_t r: order) {
        if (desires[r] == 0) {
            if (get_now() >= end_time) {
                break;
            }
            build(r);
        }
    }

    uint32_t cnt_try = 0;
    uint32_t cnt_accept = 0;
    for (uint32_t step = 0; step < PIBTS_STEPS; step++) {
        if (PIBTS_STEPS == -1) {
            if (get_now() >= end_time) {
                break;
            }
        }
        uint32_t r = rnd.get(0, robots.size() - 1);
        visited_counter++;

        cnt_accept += try_build2(r, rnd);
        cnt_try++;
        temp *= 0.99;
    }

    //Printer() << "PIBTS: " << cnt_accept << "/" << cnt_try << '\n';

    std::vector<Action> answer(robots.size());

    for (uint32_t r = 0; r < robots.size(); r++) {
        answer[r] = get_operations()[desires[r]][0];
    }
    return answer;
}

double PIBTS::get_score() const {
    return cur_score;
}
