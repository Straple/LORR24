#include <Planner/PIBT/pibts.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Containers/dsu.hpp>
#include <Objects/Environment/environment.hpp>

#include <thread>

bool PIBTS::consider() {
    if (cur_score > best_score + 1e-6) {
        best_score = cur_score;
        best_desires = desires;
    }

    return old_score + 1e-6 <= cur_score
// old_score > cur_score
#ifdef ENABLE_PIBTS_ANNEALING
           || rnd.get_d() < 1.0 / (old_score - cur_score + 5) * temp
#endif
            ;
}

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

    const auto &poses_path = get_omap().get_poses_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        if (used_pos[poses_path[depth]][depth] != -1) {
            ASSERT(used_pos[poses_path[depth]][depth] != r, "invalid used_node");
            return false;
        }
    }
    const auto &edges_path = get_omap().get_edges_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        if (used_edge[edges_path[depth]][depth] != -1) {
            ASSERT(used_edge[edges_path[depth]][depth] != r, "invalid used_edge");
            return false;
        }
    }
    return true;
}

EPath PIBTS::get_path(uint32_t r, uint32_t desired) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");
    ASSERT(validate_path(r, desired), "invalid path");

    return get_omap().get_nodes_path(robots[r].node, desired);
}

uint32_t PIBTS::get_used(uint32_t r) const {
    uint32_t answer = -1;

    auto &poses_path = get_omap().get_poses_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_pos = poses_path[depth];
        if (used_pos[to_pos][depth] != -1) {
            if (answer == -1) {
                answer = used_pos[to_pos][depth];
            } else if (answer != used_pos[to_pos][depth]) {
                return -2;
            }
        }
    }

    const auto &edges_path = get_omap().get_edges_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        if (used_edge[to_edge][depth] != -1) {
            if (answer == -1) {
                answer = used_edge[to_edge][depth];
            } else if (answer != used_edge[to_edge][depth]) {
                return -2;
            }
        }
    }
    return answer;
}

int32_t PIBTS::get_smart_dist_IMPL(uint32_t r, uint32_t desired) const {
    int32_t dist = get_dhm().get(get_omap().get_nodes_path(robots[r].node, desired).back(), robots[r].target);

    const auto &op = get_operations()[desired];
    const auto &path = get_omap().get_nodes_path(robots[r].node, desired);
    if (op.back() == Action::W) {
        uint32_t node = path[path.size() - 2];
        {
            uint32_t to = get_graph().get_to_node(node, 1);
            dist = std::min(dist, static_cast<int32_t>(get_dhm().get(to, robots[r].target)));
        }
        {
            uint32_t to = get_graph().get_to_node(node, 2);
            dist = std::min(dist, static_cast<int32_t>(get_dhm().get(to, robots[r].target)));
        }

        if (op[op.size() - 2] == Action::W) {
            node = get_graph().get_to_node(node, 1);
            node = get_graph().get_to_node(node, 1);
            dist = std::min(dist, static_cast<int32_t>(get_dhm().get(node, robots[r].target)));
        }
    }
    return dist;
}

int32_t PIBTS::get_smart_dist(uint32_t r, uint32_t desired) const {
    return smart_dist_dp[r][desired];
}

void PIBTS::update_score(uint32_t r, uint32_t desired, double &cur_score, int sign) const {
    int32_t old_dist = get_smart_dist(r, 0);// get_dhm().get(robots[r].node, robots[r].target);
    int32_t cur_dist = get_smart_dist(r, desired);
    int32_t diff = (old_dist - cur_dist);// * (old_dist - cur_dist) * (old_dist - cur_dist);
    double power = std::sqrt((max_weight - weight[r]) * 1.0 / robots.size());
    cur_score += sign * diff * power;
}

void PIBTS::add_path(uint32_t r) {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < get_operations().size(), "invalid desired");

    const auto &poses_path = get_omap().get_poses_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_pos = poses_path[depth];
        ASSERT(to_pos < used_pos.size(), "invalid to_pos");
        ASSERT(used_pos[to_pos][depth] == -1, "already used");
        used_pos[to_pos][depth] = r;
    }

    const auto &edges_path = get_omap().get_edges_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        if (to_edge) {
            ASSERT(to_edge < used_edge.size(), "invalid to_edge");
            ASSERT(used_edge[to_edge][depth] == -1, "already used");
            used_edge[to_edge][depth] = r;
        }
    }

    update_score(r, desires[r], cur_score, +1);
}

void PIBTS::remove_path(uint32_t r) {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desires[r] && desires[r] < get_operations().size(), "invalid desired");

    const auto &poses_path = get_omap().get_poses_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_pos = poses_path[depth];
        ASSERT(to_pos < used_pos.size(), "invalid to_pos");
        ASSERT(used_pos[to_pos][depth] == r, "invalid node");
        used_pos[to_pos][depth] = -1;
    }
    const auto &edges_path = get_omap().get_edges_path(robots[r].node, desires[r]);
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        uint32_t to_edge = edges_path[depth];
        if (to_edge) {
            ASSERT(to_edge < used_edge.size(), "invalid to_edge");
            ASSERT(used_edge[to_edge][depth] == r, "invalid edge");
            used_edge[to_edge][depth] = -1;
        }
    }
    update_score(r, desires[r], cur_score, -1);
}

uint32_t PIBTS::try_build(uint32_t r, uint32_t &counter, uint32_t depth) {
    if (counter > 1000 ||//
        (counter % 16 == 0 && get_now() >= end_time)) {
        counter = -1;
        return 2;
    }

    visited[r] = visited_counter;
    uint32_t old_desired = desires[r];

    for (uint32_t desired: robot_desires[r]) {
        desires[r] = desired;
        uint32_t to_r = get_used(r);
        if (to_r == -1) {
            add_path(r);
            if (consider()) {
                //Printer() << "accept try_build: " << old_score << "->" << cur_score << ", depth: " << depth << ", count: " << counter << " | " << r;
                return 1;// accepted
            } else {
                remove_path(r);
                desires[r] = old_desired;
                return 2;// not accepted
            }
        } else if (to_r != -2 && visited[to_r] != visited_counter) {
            if (rnd.get_d() < 0.1) {
                continue;
            }

            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
            ASSERT(visited[to_r] != visited_counter, "already visited");

            remove_path(to_r);
            add_path(r);

            uint32_t res = try_build(to_r, ++counter, depth + 1);
            if (res == 1) {
                //Printer() << ' ' << r;
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
        ASSERT(std::abs(old_score - cur_score) < 1e-9, "invalid rollback");
        return false;
    }
    //Printer() << '\n';
    return true;
}

uint32_t PIBTS::try_rebuild_neighbors(uint32_t id, const std::vector<uint32_t> &rids, uint32_t &counter, uint32_t depth) {
    if (id == rids.size()) {
        // все поставили
        if (consider()) {
            //Printer() << "accept try_rebuild_neighbors: " << old_score << "->" << cur_score << ", depth: " << depth << ", count: " << counter << " | ";
            //for (uint32_t r: rids) {
            //    Printer() << r << ' ';
            //}
            //Printer() << '\n';
            return 1;// accepted
        } else {
            return 2;// not accepted
        }
    }

    if (counter == -1 ||//
        (counter % 16 == 0 && get_now() >= end_time)) {
        counter = -1;
        return 2;
    }

    uint32_t r = rids[id];
    // хотим поставить путь для r
    uint32_t old_desired = desires[r];

    for (uint32_t desired: robot_desires[r]) {
        desires[r] = desired;
        uint32_t to_r = get_used(r);
        if (to_r == -1) {
            add_path(r);

            uint32_t res = try_rebuild_neighbors(id + 1, rids, ++counter, depth + 1);
            if (res == 1) {
                return res;
            } else if (res == 2) {
                remove_path(r);
                desires[r] = old_desired;
                return res;
            } else {
                remove_path(r);
            }
        }
    }
    desires[r] = old_desired;
    return 0;
}

bool PIBTS::try_rebuild_neighbors(uint32_t r) {
    ++visited_counter;
    old_score = cur_score;

    auto rids = neighbors[r];
    rids.push_back(r);

    std::shuffle(rids.begin(), rids.end(), rnd.generator);
    uint32_t size = rnd.get(1, 4);
    if (rids.size() > size) {
        rids.resize(size);
    }

    // удалим им все пути
    for (uint32_t r: rids) {
        remove_path(r);
    }

    uint32_t counter = 0;
    uint32_t res = try_rebuild_neighbors(0, rids, counter, 0);
    if (res == 0 || res == 2) {
        for (uint32_t r: rids) {
            add_path(r);
        }
        ASSERT(std::abs(old_score - cur_score) < 1e-9, "invalid rollback");
        return false;
    }
    return true;
}

uint32_t PIBTS::build(uint32_t r, uint32_t depth, uint32_t &counter) {
    if (counter == -1 ||//
        //counter > 30'000 ||//
        (counter % 16 == 0 && get_now() >= end_time)) {
        counter = -1;
        return 2;
    }

    visited[r] = visited_counter;
    uint32_t old_desired = desires[r];

    // smart version
    // (priority, desired, r2, desired2, to_r)
    /*std::vector<std::tuple<int64_t, uint32_t, uint32_t, uint32_t, uint32_t>> steps;
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
            //continue;
        }
        if (to_r != -1 && visited[to_r] == visited_counter) {
            continue;
        }
        int64_t priority = get_smart_dist(r, desired);
        steps.emplace_back(priority, desired, -1, -1, to_r);
    }

    for (uint32_t desired = 1; desired < get_operations().size(); desired++) {
        desires[r] = desired;
        if (!validate_path(r, desired)) {
            continue;
        }
        //for (uint32_t r2: neighbours[r]) {
        if(neighbours[r].empty()){
            continue;
        }
        uint32_t r2 = rnd.get(neighbours[r]);
        {
            ASSERT(r != r2, "invalid neighbours");
            uint32_t old_desired2 = desires[r2];
            remove_path(r2);
            for (uint32_t desired2 = 0; desired2 < get_operations().size(); desired2++) {
                desires[r2] = desired2;
                if (!validate_path(r2, desired2)) {
                    continue;
                }

                uint32_t used = get_used(r);
                uint32_t used2 = get_used(r2);
                if (used == -2 || used2 == -2) {
                    continue;
                }
                if (used != -1 && used2 != -1 && used != used2) {
                    continue;
                }

                uint32_t to_r = used;
                if (to_r == -1) {
                    to_r = used2;
                }

                if (to_r != -1 && visited[to_r] == visited_counter) {
                    continue;
                }

                if (to_r != -1) {
                    remove_path(to_r);
                }
                add_path(r);
                if (get_used(r2) == r) {
                    remove_path(r);
                    if (to_r != -1) {
                        add_path(to_r);
                    }
                    continue;
                }

                // здесь я могу добавить путь для r и r2 с такими desired и desired2
                // а также тут есть to_r, который мы уже построим рекурсивно

                int64_t priority = get_smart_dist(r, desired) + get_smart_dist(r2, desired2);
                steps.emplace_back(priority, desired, r2, desired2, to_r);

                remove_path(r);
                if (to_r != -1) {
                    add_path(to_r);
                }
            }
            desires[r2] = old_desired2;
            add_path(r2);
        }
    }

    std::stable_sort(steps.begin(), steps.end());

    for (auto [priority, desired, r2, desired2, to_r]: steps) {
        uint32_t old_desired2 = -1;
        if(r2 != -1){
            old_desired2 = desires[r2];
        }
        desires[r] = desired;
        if (r2 != -1) {
            remove_path(r2);
            desires[r2] = desired2;
        }

        if (to_r == -1) {
            add_path(r);
            if (r2 != -1) {
                add_path(r2);
            }
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
                if (r2 != -1) {
                    remove_path(r2);
                    desires[r2] = old_desired2;
                    add_path(r2);
                }
                return 2;// not accepted
            }
        } else {
            remove_path(to_r);
            add_path(r);
            if(r2 != -1) {
                add_path(r2);
            }

            uint32_t res = build(to_r, depth + 1, ++counter);
            if (res == 1) {
                return res;
            } else if (res == 2) {
                remove_path(r);
                if(r2 != -1) {
                    remove_path(r2);
                }
                add_path(to_r);
                desires[r] = old_desired;
                if(r2 != -1) {
                    desires[r2] = old_desired2;
                    add_path(r2);
                }
                return res;
            }

            remove_path(r);
            if(r2 != -1) {
                remove_path(r2);
            }
            add_path(to_r);
            if(r2 != -1) {
                desires[r2] = old_desired2;
                add_path(r2);
            }
        }
    }*/

    for (uint32_t desired: robot_desires[r]) {
        desires[r] = desired;
        uint32_t to_r = get_used(r);
        if (to_r == -1) {
            add_path(r);
            if (consider()) {
                //Printer() << "accept build: " << old_score << "->" << cur_score << ", depth: " << depth << ", count: " << counter << " | " << r;
                return 1;// accepted
            } else {
                remove_path(r);
                desires[r] = old_desired;
                return 2;// not accepted
            }
        } else if (to_r != -2// && visited[to_r] != visited_counter
        ) {
            if (counter > 3000 && depth >= 6) {
                continue;
            }

            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
            //ASSERT(visited[to_r] != visited_counter, "already visited");

            // TODO: у to_r может быть приоритет ниже чем у r
            // но to_r уже построен, потому что был какой-то x, который имел приоритет больше чем r
            // и этот x построил to_r
            if (desires[to_r] != 0
#ifdef ENABLE_PIBTS_TRICK
                && rnd.get_d() < visited_bound
#endif
            ) {
                continue;
            }

            remove_path(to_r);
            add_path(r);

            uint32_t res = build(to_r, depth + 1, ++counter);
            if (res == 1) {
                //Printer() << ' ' << r;
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

    visited[r] = 0;
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
        ASSERT(std::abs(old_score - cur_score) < 1e-9, "invalid rollback");
        return false;
    }
    //Printer() << '\n';
    return true;
}

PIBTS::PIBTS(const std::vector<Robot> &robots, TimePoint end_time, uint64_t seed)
    : robots(robots), end_time(end_time), rnd(seed) {

    /*if (rnd.get_d() < 0.5) {
        visited_bound = 2;
    } else {
        visited_bound = rnd.get_d(0.1, 0.9);
    }*/

    visited.resize(robots.size());
    desires.resize(robots.size());
    best_desires.resize(robots.size());

    {
        std::array<uint32_t, DEPTH> value{};
        for (uint32_t depth = 0; depth < DEPTH; depth++) {
            value[depth] = -1;
        }
        used_pos.resize(get_graph().get_zipes_size(), value);
        used_edge.resize(get_graph().get_edges_size(), value);
    }

    // build order and weight
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

    smart_dist_dp.resize(robots.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        smart_dist_dp[r].resize(get_operations().size());
        for (uint32_t desired = 0; desired < smart_dist_dp[r].size(); desired++) {
            if (validate_path(r, desired)) {
                smart_dist_dp[r][desired] = get_smart_dist_IMPL(r, desired);
            }
        }
    }

    Timer timer;

    // init neighbours
    {
        neighbors.resize(robots.size());
        // TODO: here swap depth and edge for optimize

        // used_edge[depth][edge] = robot id
        std::array<std::vector<std::vector<uint32_t>>, DEPTH> used_edge;

        // used_pos[depth][pos] = robot id
        std::array<std::vector<std::vector<uint32_t>>, DEPTH> used_pos;

        for (uint32_t depth = 0; depth < DEPTH; depth++) {
            used_pos[depth].resize(get_graph().get_zipes_size());
            used_edge[depth].resize(get_graph().get_edges_size());
        }

        for (uint32_t r = 0; r < robots.size(); r++) {
            for (uint32_t desired = 0; desired < get_operations().size(); desired++) {
                if (!validate_path(r, desired)) {
                    continue;
                }
                uint32_t source = robots[r].node;
                auto &poses_path = get_omap().get_poses_path(source, desired);
                auto &edges_path = get_omap().get_edges_path(source, desired);
                for (uint32_t depth = 0; depth < DEPTH; depth++) {
                    uint32_t to_edge = edges_path[depth];
                    uint32_t to_pos = poses_path[depth];

                    ASSERT(to_pos < used_pos[depth].size(), "invalid to_pos");
                    ASSERT(to_edge < used_edge[depth].size(), "invalid to_edge");

                    auto add = [&](std::vector<uint32_t> &vec) {
                        if (std::find(vec.begin(), vec.end(), r) == vec.end()) {
                            vec.push_back(r);
                        }
                    };

                    if (to_edge) {
                        add(used_edge[depth][to_edge]);
                    }
                    add(used_pos[depth][to_pos]);
                }
            }
        }

        for (uint32_t depth = 0; depth < DEPTH; depth++) {
            for (uint32_t edge = 1; edge < used_edge[depth].size(); edge++) {
                for (uint32_t r: used_edge[depth][edge]) {
                    for (uint32_t r2: used_edge[depth][edge]) {
                        if (r != r2 &&
                            std::find(neighbors[r].begin(), neighbors[r].end(), r2) == neighbors[r].end()) {
                            neighbors[r].push_back(r2);
                        }
                    }
                }
            }
            for (uint32_t pos = 1; pos < used_pos[depth].size(); pos++) {
                for (uint32_t r: used_pos[depth][pos]) {
                    for (uint32_t r2: used_pos[depth][pos]) {
                        if (r != r2 &&
                            std::find(neighbors[r].begin(), neighbors[r].end(), r2) == neighbors[r].end()) {
                            neighbors[r].push_back(r2);
                        }
                    }
                }
            }
        }
    }

    for (uint32_t r = 0; r < robots.size(); r++) {
        desires[r] = 0;
        add_path(r);
    }

    robot_desires.resize(robots.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        // (priority, desired)
        std::vector<std::pair<int64_t, uint32_t>> steps;
        for (uint32_t desired = 1; desired < get_operations().size(); desired++) {
            if (!validate_path(r, desired)) {
                continue;
            }
            int64_t priority = get_smart_dist(r, desired);
            steps.emplace_back(priority, desired);
        }
        std::stable_sort(steps.begin(), steps.end());
        for (auto [priority, desired]: steps) {
            robot_desires[r].push_back(desired);
        }
    }
}

void PIBTS::simulate_pibt() {
    // PIBT
    /*const bool skip_with_init_desired = rnd.get_d() < 0.5;
    for (uint32_t r: order) {
        if (get_now() >= end_time) {
            break;
        }
        if (desires[r] != 0 && skip_with_init_desired) {
            continue;
        }
        build(r);
    }

    //Printer() << "PIBTS: " << cur_score;
    std::vector<std::thread> threads(2);
    for (uint32_t thr = 0; thr < threads.size(); thr++) {
        threads[thr] = std::thread(
                [&](uint32_t thr) {
                    do_work(thr);
                },
                thr);
    }
    for (uint32_t thr = 0; thr < threads.size(); thr++) {
        threads[thr].join();
    }*/
    //Printer() << '\n';
    //Printer() << "PIBTS version: " << version << '\n';

    // TODO: подобрать 0.5
    const bool skip_with_init_desired =
#ifdef ENABLE_PIBTS_TRICK
            rnd.get_d() < 0.5;
#else
            false;
#endif
    temp = 0;
    for (uint32_t r: order) {
        if (get_now() >= end_time) {
            break;
        }
        if (desires[r] != 0 && skip_with_init_desired) {
            continue;
        }
        build(r);
    }

    double score_after_build = best_score;

    temp = 1;
    for (uint32_t step = 0; step < PIBTS_STEPS && get_now() < end_time; step++) {
        uint32_t r = rnd.get(0, robots.size() - 1);
        if (rnd.get_d() < 0.5) {
            try_build(r);
        } else {
            try_rebuild_neighbors(r);
        }
        temp *= 0.999;
    }

    /*Printer() << "PIBTS: " << score_after_build << "->" << best_score << '\n';

    if (best_score > cur_score + 0.1) {
        Printer() << "best better then cur\n";
    }*/
}

double PIBTS::get_score() const {
    return cur_score;
}

std::vector<Action> PIBTS::get_actions() const {
    std::vector<Action> answer(robots.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        answer[r] = get_operations()[best_desires[r]][0];
        if (best_desires[r] == 0) {
            auto dist = std::min({get_dhm().get(get_graph().get_to_node(robots[r].node, 1), robots[r].target),
                                  get_dhm().get(get_graph().get_to_node(robots[r].node, 2), robots[r].target),
                                  get_dhm().get(get_graph().get_to_node(robots[r].node, 3), robots[r].target)});
            if (dist == get_dhm().get(get_graph().get_to_node(robots[r].node, 1), robots[r].target)) {
                answer[r] = Action::CR;
            } else if (dist == get_dhm().get(get_graph().get_to_node(robots[r].node, 2), robots[r].target)) {
                answer[r] = Action::CCR;
            } else {
                answer[r] = Action::W;
            }
        }
    }
    return answer;
}

std::vector<uint32_t> PIBTS::get_desires() const {
    return best_desires;
}

std::vector<int64_t> PIBTS::get_changes() const {
    std::vector<int64_t> answer(robots.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        answer[r] = get_smart_dist(r, 0) - get_smart_dist(r, best_desires[r]);
    }
    return answer;
}

double PIBTS::get_best_score() const {
    return best_score;
}
