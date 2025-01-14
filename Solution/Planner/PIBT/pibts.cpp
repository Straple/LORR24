#include <Planner/PIBT/pibts.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Containers/dsu.hpp>
#include <Objects/Environment/environment.hpp>

#include <thread>

bool PIBTS::validate_path(uint32_t r, uint32_t desired) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");
    uint32_t node = robots[r].node;
    return get_omap().get_poses_path(node, desired)[0] > 0;
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

    uint32_t source = robots[r].node;
    auto &poses_path = get_omap().get_poses_path(source, desired);
    auto &edges_path = get_omap().get_edges_path(source, desired);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        uint32_t to_pos = poses_path[depth];

        uint32_t used_edge_value = state.used_edge[depth].count(to_edge) ? state.used_edge[depth].at(to_edge)
                                                                         : used_edge[depth][to_edge];

        uint32_t used_pos_value = state.used_pos[depth].count(to_pos) ? state.used_pos[depth].at(to_pos)
                                                                      : used_pos[depth][to_pos];

        if (used_edge_value != -1) {
            ASSERT(used_edge_value != r || state.version != version, "invalid used_edge");
            return false;
        }

        if (used_pos_value != -1) {
            ASSERT(used_pos_value != r || state.version != version, "invalid used_node");
            return false;
        }
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
    uint32_t answer = -1;

    uint32_t source = robots[r].node;
    auto &poses_path = get_omap().get_poses_path(source, desires[r]);
    auto &edges_path = get_omap().get_edges_path(source, desires[r]);

    //uint32_t start_pos = get_graph().get_pos(source).get_pos();
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        uint32_t to_pos = poses_path[depth];

        //if (cluster_id[to_pos] != cluster_id[start_pos]) {
        //    answer = -2;
        //    break;
        //}

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
    ASSERT(validate_path(r, desired), "invalid path");

    uint32_t answer = -1;
    uint32_t source = robots[r].node;
    auto &poses_path = get_omap().get_poses_path(source, desired);
    auto &edges_path = get_omap().get_edges_path(source, desired);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        uint32_t to_pos = poses_path[depth];

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
    }
    return answer;
}

int64_t PIBTS::get_smart_dist(uint32_t r, uint32_t desired) const {
    int64_t dist = get_dhm().get(get_omap().get_nodes_path(robots[r].node, desired).back(), robots[r].target);

    const auto &op = get_operations()[desired];
    const auto &path = get_omap().get_nodes_path(robots[r].node, desired);
    if (op.back() == Action::W) {
        uint32_t node = path[path.size() - 2];
        {
            uint32_t to = get_graph().get_to_node(node, 1);
            dist = std::min(dist, static_cast<int64_t>(get_dhm().get(to, robots[r].target)));
        }
        {
            uint32_t to = get_graph().get_to_node(node, 2);
            dist = std::min(dist, static_cast<int64_t>(get_dhm().get(to, robots[r].target)));
        }

        if (op[op.size() - 2] == Action::W) {
            node = get_graph().get_to_node(node, 1);
            node = get_graph().get_to_node(node, 1);
            dist = std::min(dist, static_cast<int64_t>(get_dhm().get(node, robots[r].target)));
        }
    }
    return dist;
}

void PIBTS::update_score(uint32_t r, uint32_t desired, double &cur_score, int sign) const {
    int64_t old_dist = get_dhm().get(robots[r].node, robots[r].target);
    int64_t cur_dist = get_smart_dist(r, desired);
    int64_t diff = (old_dist - cur_dist);// * (old_dist - cur_dist) * (old_dist - cur_dist);
    double power = (static_cast<int32_t>(robots.size()) - weight[r]) * 1.0 / robots.size();
    cur_score += sign * diff * power;
    //double power = (max_weight - weight[r]) * 1.0 / max_weight;
}

void PIBTS::add_path(uint32_t r) {
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

        ASSERT(used_edge[depth][to_edge] == -1, "already used");
        used_edge[depth][to_edge] = r;

        ASSERT(used_pos[depth][to_pos] == -1, "already used");
        used_pos[depth][to_pos] = r;
    }

    update_score(r, desires[r], cur_score, +1);
}

void PIBTS::add_path(uint32_t r, State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");

    uint32_t source = robots[r].node;
    auto &poses_path = get_omap().get_poses_path(source, desired);
    auto &edges_path = get_omap().get_edges_path(source, desired);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        uint32_t to_pos = poses_path[depth];

        ASSERT(to_pos < used_pos[depth].size(), "invalid to_pos");
        ASSERT(to_edge && to_edge < used_edge[depth].size(), "invalid to_edge");

        uint32_t used_edge_value = state.used_edge[depth].count(to_edge) ? state.used_edge[depth].at(to_edge)
                                                                         : used_edge[depth][to_edge];

        uint32_t used_pos_value = state.used_pos[depth].count(to_pos) ? state.used_pos[depth].at(to_pos)
                                                                      : used_pos[depth][to_pos];

        ASSERT(used_edge_value == -1 || state.version != version, "already used");
        state.used_edge[depth][to_edge] = r;

        ASSERT(used_pos_value == -1 || state.version != version, "already used");
        state.used_pos[depth][to_pos] = r;
    }

    update_score(r, desired, state.cur_score, +1);
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

    update_score(r, desires[r], cur_score, -1);
}

void PIBTS::remove_path(uint32_t r, State &state) const {
    uint32_t desired = state.desires.count(r) ? state.desires.at(r) : desires[r];
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");

    uint32_t source = robots[r].node;
    auto &poses_path = get_omap().get_poses_path(source, desired);
    auto &edges_path = get_omap().get_edges_path(source, desired);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        uint32_t to_pos = poses_path[depth];

        ASSERT(to_pos < used_pos[depth].size(), "invalid to_pos");
        ASSERT(to_edge && to_edge < used_edge[depth].size(), "invalid to_edge");

        uint32_t used_edge_value = state.used_edge[depth].count(to_edge) ? state.used_edge[depth].at(to_edge)
                                                                         : used_edge[depth][to_edge];

        uint32_t used_pos_value = state.used_pos[depth].count(to_pos) ? state.used_pos[depth].at(to_pos)
                                                                      : used_pos[depth][to_pos];

        ASSERT(used_edge_value == r || state.version != version, "invalid edge");
        state.used_edge[depth][to_edge] = -1;

        ASSERT(used_pos_value == r || state.version != version, "invalid node");
        state.used_pos[depth][to_pos] = -1;
    }

    update_score(r, desired, state.cur_score, -1);
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

bool PIBTS::try_build_state(uint32_t r, State &state, uint32_t &counter, Randomizer &rnd) const {
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

            if (try_build_state(to_r, state, ++counter, rnd)) {
                return true;
            }

            remove_path(r, state);
            add_path(to_r, state);
        }
    }

    state.desires.erase(r);
    return false;
}

bool PIBTS::try_build_state(uint32_t r, Randomizer &rnd) {
    State state;
    state.cur_score = cur_score;
    remove_path(r, state);
    uint32_t counter = 0;
    if (!try_build_state(r, state, counter, rnd)) {
        return false;
    }

    // TODO: annealing
    if (cur_score <= state.cur_score) {
        flush_state(state);
        return true;
    }
    return false;
}

uint32_t PIBTS::try_build(uint32_t r, uint32_t &counter, uint32_t depth) {
    if (counter == -1 || //
        counter > 1000 ||//
        (counter % 256 == 0 && get_now() >= end_time)) {

        counter = -1;
        return 2;
    }

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
        if (to_r == -2 || (to_r != -1 && visited[to_r] == visited_counter)) {
            continue;
        }
        int64_t priority = get_smart_dist(r, desired);
        steps.emplace_back(priority, desired);
    }

    std::stable_sort(steps.begin(), steps.end());

    for (auto [_, desired]: steps) {
        desires[r] = desired;
        if (is_free_path(r)) {
            add_path(r);
            if (old_score - 1e-6 <= cur_score
            // old_score > cur_score
#ifdef ENABLE_PIBTS_ANNEALING
                || rnd.get_d() < 1.0 / (old_score - cur_score + 5) * temp
#endif
            ) {
                return 1;// accepted
            } else {
                remove_path(r);
                desires[r] = old_desired;
                return 2;// not accepted
            }
        } else {
            if (counter > 200 && depth >= 6) {
                continue;
            }

            uint32_t to_r = get_used(r);
            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
            ASSERT(visited[to_r] != visited_counter, "already visited");

            remove_path(to_r);
            add_path(r);

            uint32_t res = try_build(to_r, ++counter, depth + 1);
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

bool PIBTS::try_build(uint32_t r) {
    ++visited_counter;
    old_score = cur_score;
    remove_path(r);
    uint32_t counter = 0;
    uint32_t res = try_build(r, counter, 0);
    if (res == 0 || res == 2) {
        add_path(r);
        return false;
    }
    return true;
}

uint32_t PIBTS::build(uint32_t r, uint32_t depth, uint32_t &counter) {
    if (counter == -1 ||//
        //counter > 30'000 ||//
        (counter % 256 == 0 && get_now() >= end_time)) {

        counter = -1;
        return 2;
    }

    uint32_t old_desired = desires[r];

    // (priority, desired)
    std::vector<std::pair<int64_t, uint32_t>> steps;
    for (uint32_t desired = 0; desired < get_operations().size(); desired++) {
        desires[r] = desired;
        if (!validate_path(r, desires[r])) {
            continue;
        }
        uint32_t to_r = get_used(r);
        if (to_r == -2) {
            continue;
        }
        if (to_r != -1 && desires[to_r] != 0) {
            //continue;
        }
        int64_t priority = get_smart_dist(r, desired);
        steps.emplace_back(priority, desired);
    }

    std::stable_sort(steps.begin(), steps.end());

    for (auto [priority, desired]: steps) {
        desires[r] = desired;
        if (is_free_path(r)) {
            // отлично! там никого нет
            add_path(r);
            if (old_score - 1e-6 <= cur_score
            // old_score > cur_score
#ifdef ENABLE_PIBTS_ANNEALING
                || rnd.get_d() < 1.0 / (old_score - cur_score + 5) * temp
#endif
            ) {
                return 1;// accepted
            } else {
                remove_path(r);
                desires[r] = old_desired;
                return 2;// not accepted
            }
        } else {
            if (counter > 3000 && depth >= 6) {
                continue;
            }

            uint32_t to_r = get_used(r);
            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
            //ASSERT(desires[to_r] == 0, "invalid desires");

            if (desires[to_r] != 0
#ifdef ENABLE_PIBTS_TRICK
                && rnd.get_d() < 0.8
#endif
            ) {
                continue;
            }

            remove_path(to_r);
            add_path(r);

            uint32_t res = build(to_r, depth + 1, ++counter);
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
    return false;
}

bool PIBTS::build(uint32_t r) {
    ++visited_counter;
    old_score = cur_score;
    remove_path(r);
    uint32_t counter = 0;
    uint32_t res = build(r, 0, counter);
    if (res == 0 || res == 2) {
        add_path(r);
        return false;
    }
    return true;
}

bool PIBTS::build_state(uint32_t r, uint32_t depth, uint32_t &counter, State &state) {
    if (counter == -1 ||  //
        counter > 5'000 ||//
        (counter % 256 == 0 && get_now() >= end_time)) {

        counter = -1;
        return false;
    }

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

        if (is_free_path(r, state)) {
            add_path(r, state);
            return true;
        } else {
            uint32_t to_r = get_used(r, state);
            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
            uint32_t to_desired = state.desires.count(to_r) ? state.desires[to_r] : desires[to_r];
            if (to_desired != 0) {
                continue;
            }
            remove_path(to_r, state);
            add_path(r, state);

            if (build_state(to_r, depth + 1, ++counter, state)) {
                return true;
            }

            remove_path(r, state);
            add_path(to_r, state);
        }
    }

    state.desires.erase(r);
    return false;
}

bool PIBTS::build_state(uint32_t r) {
    ASSERT(false, "TODO");
    return true;
    /*State best_state;
    best_state.cur_score = cur_score;

    State start;
    start.cur_score = cur_score;
    remove_path(r, start);

    for (uint32_t depth = 0; depth < 10; depth++) {
        auto s = start;
        uint32_t counter = 0;
        if (build_state(r, 0, counter, s)) {
            double best = best_state.cur_score * 100 - best_state.desires.size();
            double cur = s.cur_score * 100 - s.desires.size();
            if (best < cur) {
                best_state = s;
            }
        }
    }

    flush_state(best_state);
    return true;

    if (best_state.cur_score > cur_score) {
        flush_state(best_state);
        return true;
    } else {
        return false;
    }*/
}

void PIBTS::build_clusters() {
    static Randomizer rnd;

    std::vector<uint32_t> order(robots.size());
    std::iota(order.begin(), order.end(), 0);
    //std::shuffle(order.begin(), order.end(), rnd.generator);
    std::sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
        return get_robots_handler().get_robot(lhs).priority < get_robots_handler().get_robot(rhs).priority;
    });

    DSU dsu(get_map().get_size());

    constexpr uint32_t CLUSTER_SIZE = 150;

    // (priority, desired, r)
    std::vector<std::tuple<int64_t, uint32_t, uint32_t>> pool;

    for (uint32_t r: order) {
        Position start = get_graph().get_pos(robots[r].node);

        for (uint32_t desired = 0; desired < get_operations().size(); desired++) {
            if (validate_path(r, desired)) {
                auto path = get_path(r, desired);

                int64_t priority = get_dhm().get(path.back(), robots[r].target);
                pool.emplace_back(priority, desired, r);
            }
        }
    }
    //std::stable_sort(pool.begin(), pool.end());
    std::shuffle(pool.begin(), pool.end(), rnd.generator);

    {
        for (auto [_, desired, r]: pool) {
            auto operation = get_operations()[desired];
            bool ok = true;
            Position start = get_graph().get_pos(robots[r].node);
            Position p = start;
            std::set<std::pair<uint32_t, uint32_t>> S;
            for (auto action: operation) {
                p = p.simulate_action(action);
                if (!p.is_valid()) {
                    ok = false;
                    break;
                }
                if (dsu.get(p.get_pos()) != dsu.get(start.get_pos())) {
                    S.insert({p.get_pos(), dsu.get_size(p.get_pos())});
                }
            }
            uint32_t total_size = dsu.get_size(start.get_pos());
            for (auto [_, size]: S) {
                total_size += size;
            }
            if (total_size > CLUSTER_SIZE) {
                continue;
            }

            if (ok) {
                Position p = start;
                for (auto action: operation) {
                    p = p.simulate_action(action);
                    dsu.uni(p.get_pos(), start.get_pos());
                }
            }
        }
    }
    std::set<std::pair<uint32_t, uint32_t>> S;
    for (uint32_t pos = 1; pos < get_map().get_size(); pos++) {
        if (get_map().is_free(pos)) {
            S.insert({dsu.get_size(pos), dsu.get(pos)});
        }
    }
    while (S.size() >= 2 && S.begin()->first + (++S.begin())->first <= CLUSTER_SIZE) {
        auto a = *S.begin();
        S.erase(S.begin());
        auto b = *S.begin();
        S.erase(S.begin());
        dsu.uni(a.second, b.second);
        S.insert({dsu.get_size(a.second), dsu.get(a.second)});
    }
    for (uint32_t pos = 1; pos < get_map().get_size(); pos++) {
        if (!get_map().is_free(pos)) {
            dsu.uni(0, pos);
        }
    }

    cluster_id.resize(get_map().get_size());
    for (uint32_t pos = 1; pos < cluster_id.size(); pos++) {
        cluster_id[pos] = dsu.get(pos);
    }

    /*{
        static uint32_t id = 0;
        std::ofstream output("Clusters/cluster" + std::to_string(id) + ".txt");
        id++;
        output << get_map().get_rows() << ' ' << get_map().get_cols() << '\n';
        std::unordered_map<uint32_t, uint32_t> kek;
        for (uint32_t pos = 0; pos < cluster_id.size(); pos++) {
            if(!kek.count(cluster_id[pos])){
                kek[dsu.get(pos)] = kek.size();
            }
        }
        for (uint32_t x = 0; x < get_map().get_rows(); x++) {
            for (uint32_t y = 0; y < get_map().get_cols(); y++) {
                if (y != 0) {
                    output << ' ';
                }
                uint32_t val = kek[cluster_id[x * get_map().get_cols() + y + 1]];
                if (val != 0) {
                    val += 3;
                }
                output << val;
            }
            output << '\n';
        }
    }*/
}

uint32_t PIBTS::parallel_build(uint32_t r, uint32_t depth, uint32_t &counter, State &state) const {
    if (state.version != version) {
        return 2;// new version
    }
    if (counter == -1 || counter > 5'000) {
        return 2;
    }
    if (get_now() >= end_time) {
        counter = -1;
        return 2;
    }
    // (priority, desired)
    std::vector<std::pair<int64_t, uint32_t>> steps;
    for (uint32_t desired = 0; desired < get_operations().size(); desired++) {
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

        if (is_free_path(r, state)) {
            add_path(r, state);
            return 1;
        } else {
            if (counter > 3'000 && depth >= 6) {
                continue;
            }
            uint32_t to_r = get_used(r, state);
            if (state.version != version) {
                return 2;
            }
            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
            if (state.desires.count(to_r)) {
                continue;
            }
            remove_path(to_r, state);
            add_path(r, state);

            uint32_t res = parallel_build(to_r, depth + 1, ++counter, state);
            if (res != 0) {
                return res;
            }

            remove_path(r, state);
            add_path(to_r, state);
        }
    }

    state.desires.erase(r);
    return 0;
}

bool PIBTS::parallel_build(uint32_t r) {
    State s;
    s.version = version;
    if (s.version & 1) {
        return false;
    }
    s.cur_score = cur_score;
    remove_path(r, s);
    uint32_t counter = 0;
    uint32_t res = parallel_build(r, 0, counter, s);
    if (res == 0 || res == 2) {
        return false;
    }
    double old_score = cur_score;

    if (s.cur_score <= old_score + 1e-6) {
        return false;
    }

    if (!version.compare_exchange_strong(s.version, s.version + 1)) {
        return false;
    }
    flush_state(s);
    Printer() << "->" << cur_score;
    ++version;

    return true;
}

void PIBTS::do_work(uint32_t thr) {
    Randomizer rnd(this->rnd.get() + thr * 1283);
    for (uint32_t step = 0; step < PIBTS_STEPS && get_now() < end_time; step++) {
        // waiting for other thread flush state
        while (version & 1) {}

        uint32_t r = rnd.get(0, robots.size() - 1);
        parallel_build(r);
    }
}

PIBTS::PIBTS(const std::vector<Robot> &robots, TimePoint end_time, uint64_t seed)
    : robots(robots), end_time(end_time), rnd(seed) {

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

        uint32_t prev = 0;
        uint32_t w = 0;
        for (uint32_t r: order) {
            if (prev != robots[r].priority) {
                w++;
            }
            weight[r] = w;
            prev = robots[r].priority;
        }
        max_weight = w + 1;

        /*for (uint32_t i = 0; i < robots.size(); i++) {
            weight[order[i]] = i;
        }*/
    }

    for (uint32_t r = 0; r < robots.size(); r++) {
        add_path(r);
    }

    //build_clusters();
}

void PIBTS::simulate_pibt() {
    // PIBT
    /*const bool skip_with_init_desired = true;//rnd.get_d() < 0.5;
    for (uint32_t r: order) {
        if (get_now() >= end_time) {
            break;
        }
        if (desires[r] != 0 && skip_with_init_desired) {
            //continue;
        }
        build(r);
    }

    Printer() << "PIBTS: " << cur_score;
    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(
                [&](uint32_t thr) {
                    do_work(thr);
                },
                thr);
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }
    Printer() << '\n';*/

    const bool skip_with_init_desired =
#ifdef ENABLE_PIBTS_TRICK
            rnd.get_d() < 0.5;
#else
            false;
#endif
    for (uint32_t r: order) {
        if (get_now() >= end_time) {
            break;
        }
        if (desires[r] != 0 && skip_with_init_desired) {
            continue;
        }
        build(r);
    }

    for (uint32_t step = 0; step < PIBTS_STEPS && get_now() < end_time; step++) {
        uint32_t r = rnd.get(0, robots.size() - 1);
        try_build(r);
        temp *= 0.999;
    }
}

double PIBTS::get_score() const {
    return cur_score;
}

std::vector<Action> PIBTS::get_actions() const {
    std::vector<Action> answer(robots.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        answer[r] = get_operations()[desires[r]][0];
    }
    return answer;
}
