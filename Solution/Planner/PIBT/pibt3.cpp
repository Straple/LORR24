#include <Planner/PIBT/pibt3.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/randomizer.hpp>
#include <Objects/Containers/dsu.hpp>
#include <Objects/Environment/environment.hpp>
#include <unordered_set>

bool PIBT3::validate_path_IMPL(uint32_t r) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= robots[r].desired && robots[r].desired < actions.size(), "invalid desired");

    uint32_t node = robots[r].start_node;
    auto &path = actions[robots[r].desired];
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

bool PIBT3::validate_path(uint32_t r) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= robots[r].desired && robots[r].desired < actions.size(), "invalid desired");

    uint32_t node = robots[r].start_node;
    auto &path = actions[robots[r].desired];
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);

        if (!to_node || !to_edge) {
            return false;
        }

        if (cluster_id[get_graph().get_pos(robots[r].start_node).get_pos()] != cluster_id[get_graph().get_pos(to_node).get_pos()]) {
            return false;
        }

        node = to_node;
    }
    return true;
}

bool PIBT3::check_path(uint32_t r) const {
    if (!validate_path(r)) {
        return false;
    }
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= robots[r].desired && robots[r].desired < actions.size(), "invalid desired");

    uint32_t node = robots[r].start_node;
    auto &path = actions[robots[r].desired];
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

std::array<uint32_t, DEPTH> PIBT3::get_path(uint32_t r) const {
    uint32_t node = robots[r].start_node;
    auto &path = actions[robots[r].desired];
    std::array<uint32_t, DEPTH> result{};
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);

        result[depth] = to_node;
        node = to_node;
    }
    return result;
}

uint32_t PIBT3::get_used(uint32_t r) const {
    uint32_t node = robots[r].start_node;
    auto &path = actions[robots[r].desired];
    std::set<uint32_t> answer;
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        auto action = path[depth];
        uint32_t to_node = get_graph().get_to_node(node, action);
        uint32_t to_edge = get_graph().get_to_edge(node, action);
        uint32_t to_pos = get_graph().get_pos(to_node).get_pos();

        if (used_edge[depth][to_edge] != -1) {
            answer.insert(used_edge[depth][to_edge]);
        }

        if (used_pos[depth][to_pos] != -1) {
            answer.insert(used_pos[depth][to_pos]);
        }

        node = to_node;
    }
    if (answer.size() > 1) {
        return -2;
    }
    if (answer.empty()) {
        return -1;
    }
    return *answer.begin();
}

void PIBT3::add_path(uint32_t r) {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= robots[r].desired && robots[r].desired < actions.size(), "invalid desired");

    uint32_t node = robots[r].start_node;
    ASSERT(node != 0, "invalid start node");
    auto &path = actions[robots[r].desired];
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
}

void PIBT3::remove_path(uint32_t r) {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= robots[r].desired && robots[r].desired < actions.size(), "invalid desired");

    uint32_t node = robots[r].start_node;
    auto &path = actions[robots[r].desired];
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
}

bool PIBT3::build(uint32_t r, uint32_t depth, uint32_t& counter) {
    if (finish_time) {
        return false;
    }

    counter_call_build++;
    if (counter_call_build % 256 == 0 && get_now() > end_time) {
        finish_time = true;
        return false;
    }

    uint32_t old_desired = robots[r].desired;

    // (priority, desired)
    std::vector<std::pair<int64_t, uint32_t>> steps;
    for (uint32_t desired = 1; desired < actions.size(); desired++) {
        robots[r].desired = desired;
        if (validate_path(r) && get_used(r) != -2) {
            auto path = get_path(r);

            int64_t priority = get_hm().get(path.back(), get_robots_handler().get_robot(r).target);

            steps.emplace_back(priority, desired);
        }
    }

    std::stable_sort(steps.begin(), steps.end());

    for (auto [priority, desired]: steps) {
        robots[r].desired = desired;
        if (check_path(r)) {
            // отлично! там никого нет
            add_path(r);
            return true;
        } else {
            // о нет! там кто-то есть

            if (counter > 100000 && depth > 10) {
                continue;
            }

            uint32_t to_r = get_used(r);
            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
            if (robots[to_r].desired != 0) {
                continue;
            }
            remove_path(to_r);
            add_path(r);

            if (build(to_r, depth + 1, ++counter)) {
                return true;
            }

            remove_path(r);
            add_path(to_r);
        }
    }

    robots[r].desired = old_desired;
    return false;
}

void PIBT3::build_clusters() {
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
        Position start = get_graph().get_pos(robots[r].start_node);

        for (uint32_t desired = 0; desired < actions.size(); desired++) {
            robots[r].desired = desired;
            if (validate_path_IMPL(r)) {
                auto path = get_path(r);

                int64_t priority = get_hm().get(path.back(), get_robots_handler().get_robot(r).target);
                pool.emplace_back(priority, desired, r);
            }
        }
        robots[r].desired = 0;
    }
    //std::stable_sort(pool.begin(), pool.end());
    std::shuffle(pool.begin(), pool.end(), rnd.generator);
    {
        for (auto [_, desired, r]: pool) {
            auto operation = actions[desired];
            bool ok = true;
            Position start = get_graph().get_pos(robots[r].start_node);
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

PIBT3::PIBT3() {
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        used_pos[depth].resize(get_map().get_size(), -1);
        used_edge[depth].resize(get_graph().get_edges_size(), -1);
    }

    robots.resize(get_robots_handler().size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        robots[r].start_node = get_robots_handler().get_robot(r).node;
        add_path(r);
    }

    build_clusters();
}

std::vector<Action> PIBT3::solve(const std::vector<uint32_t> &order, const TimePoint end_time) {
    this->end_time = end_time;
    Timer timer;
    // PIBT
    for (uint32_t r: order) {
        if (get_now() > end_time) {
            break;
        }
        if (robots[r].desired == 0) {
            remove_path(r);
            uint32_t counter = 0;
            if (!build(r, 0, counter)) {
                add_path(r);
            }
        }
    }

    std::vector<Action> answer(robots.size());

    for (uint32_t r = 0; r < robots.size(); r++) {
        answer[r] = actions[robots[r].desired][0];
    }

    Printer() << "PIBT3: " << timer << '\n';
    return answer;
}
