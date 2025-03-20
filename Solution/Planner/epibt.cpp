#include <Planner/epibt.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/heuristic_matrix.hpp>
#include <Objects/Environment/operations_map.hpp>

#include <thread>

bool EPIBT::validate_path(uint32_t r, uint32_t desired) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");
    uint32_t node = robots[r].node;
    return get_omap().get_poses_path(node, desired)[0] > 0;
}

bool EPIBT::is_free_path(uint32_t r) const {
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

const EPath &EPIBT::get_path(uint32_t r, uint32_t desired) const {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");
    ASSERT(validate_path(r, desired), "invalid path");

    return get_omap().get_nodes_path(robots[r].node, desired);
}

uint32_t EPIBT::get_used(uint32_t r) const {
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

int64_t EPIBT::get_smart_dist_IMPL(uint32_t r, uint32_t desired) const {
    const auto &op = get_operations()[desired];
    const auto &path = get_omap().get_nodes_path(robots[r].node, desired);

    const uint32_t target = robots[r].target;

    int64_t dist = get_hm().get(path.back(), target);

    if (op.back() == Action::W) {
        uint32_t node = path[path.size() - 2];
        {
            uint32_t to = get_graph().get_to_node(node, 1);
            dist = std::min(dist, static_cast<int64_t>(get_hm().get(to, target)));
        }
        {
            uint32_t to = get_graph().get_to_node(node, 2);
            dist = std::min(dist, static_cast<int64_t>(get_hm().get(to, target)));
        }

        if (op[op.size() - 2] == Action::W) {
            uint32_t to = node;
            to = get_graph().get_to_node(to, 1);
            to = get_graph().get_to_node(to, 1);
            dist = std::min(dist, static_cast<int64_t>(get_hm().get(to, target)));
        }
    }

    // [KEK]: если мы проходим по таргету, то мы должны это делать как можно раньше. 7200 -> 7297
    for (uint32_t d = 0; d < DEPTH; d++) {
        if (get_graph().get_pos(path[d]).get_pos() == target) {
            dist = d;
            dist = -dist;
        }
    }

    dist = dist * 50 - desired;

    // стой и никому не мешай
    if (robots[r].is_disable()) {
        FAILED_ASSERT("disable is deprecated");
        dist = desired;
    }
    return dist;
}

int64_t EPIBT::get_smart_dist(uint32_t r, uint32_t desired) const {
    return smart_dist_dp[r][desired];
}

void EPIBT::update_score(uint32_t r, uint32_t desired, int sign) {
    int32_t diff = get_smart_dist(r, 0) - get_smart_dist(r, desired);
    cur_score += sign * diff * robot_power[r];
}

void EPIBT::add_path(uint32_t r) {
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

    update_score(r, desires[r], +1);
}

void EPIBT::remove_path(uint32_t r) {
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
    update_score(r, desires[r], -1);
}

bool EPIBT::build(uint32_t r, uint32_t depth, uint32_t &counter) {
    if (counter == -1 || (counter % 32 == 0 && get_now() > end_time)) {
        counter = -1;
        return false;
    }

    uint32_t old_desired = desires[r];

    for (uint32_t desired: robot_desires[r]) {
        desires[r] = desired;
        uint32_t to_r = get_used(r);
        if (to_r == -1) {
            // отлично! там никого нет
            add_path(r);
            return true;
        } else if (to_r != -2) {
            // о нет! там кто-то есть

            if (counter > 10'000) {
                continue;
            }

            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r: " + std::to_string(to_r));
            if (desires[to_r] != 0) {
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

    desires[r] = old_desired;
    return false;
}

EPIBT::EPIBT(const std::vector<Robot> &robots, TimePoint end_time)
    : robots(robots), end_time(end_time), desires(robots.size()) {

    ETimer timer;

    {
        order.resize(robots.size());
        iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
            return robots[lhs].priority < robots[rhs].priority;
        });

        std::vector<int32_t> weight(robots.size());

        for (uint32_t i = 0; i < robots.size(); i++) {
            weight[order[i]] = i;
        }
        int32_t max_weight = robots.size() + 1;

        robot_power.resize(robots.size());
        const double workload = robots.size() * 1.0 / get_map().get_count_free();
        for (uint32_t r = 0; r < robots.size(); r++) {
            double power = (max_weight - weight[r]) * 1.0 / max_weight;
            if (robots[r].is_disable()) {
                FAILED_ASSERT("robot must be not disable");
                power = 0;
            }
            power = power * power;
            robot_power[r] = power;
        }

        std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
            return std::tie(robot_power[lhs], lhs) > std::tie(robot_power[rhs], rhs);
        });
    }

    {
        std::array<uint32_t, DEPTH> value{};
        for (uint32_t depth = 0; depth < DEPTH; depth++) {
            value[depth] = -1;
        }
        used_pos.resize(get_graph().get_zipes_size(), value);
        used_edge.resize(get_graph().get_edges_size(), value);
    }

    // init smart_dist_dp
    {
        smart_dist_dp.resize(robots.size(), std::vector<int64_t>(get_operations().size()));

        auto do_work = [&](uint32_t thr) {
            for (uint32_t r = thr; r < robots.size(); r += THREADS) {
                for (uint32_t desired = 0; desired < get_operations().size(); desired++) {
                    if (!validate_path(r, desired)) {
                        continue;
                    }
                    smart_dist_dp[r][desired] = get_smart_dist_IMPL(r, desired);
                }
            }
        };

        std::vector<std::thread> threads(THREADS);
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            threads[thr] = std::thread(do_work, thr);
        }
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            threads[thr].join();
        }
    }

    // init robot_desires
    {
        robot_desires.resize(robots.size());

        auto do_work = [&](uint32_t thr) {
            for (uint32_t r = thr; r < robots.size(); r += THREADS) {
                // (priority, desired)
                std::vector<std::pair<int64_t, uint32_t>> steps;
                for (uint32_t desired = 1; desired < get_operations().size(); desired++) {
                    if (!validate_path(r, desired)) {
                        continue;
                    }
                    int64_t priority = get_smart_dist(r, desired);
                    steps.emplace_back(priority, desired);
                }
                std::reverse(steps.begin(), steps.end());
                std::stable_sort(steps.begin(), steps.end());
                for (auto [priority, desired]: steps) {
                    robot_desires[r].push_back(desired);
                }
            }
        };

        std::vector<std::thread> threads(THREADS);
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            threads[thr] = std::thread(do_work, thr);
        }
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            threads[thr].join();
        }
    }

    for (uint32_t r = 0; r < robots.size(); r++) {
        add_path(r);
    }

    PRINT(Printer() << "[EPIBT] create: " << timer << '\n';);
}

void EPIBT::solve() {
    uint32_t cnt_completed = 0;
    for (uint32_t r: order) {
        if (get_now() > end_time) {
            break;
        }
        cnt_completed++;
        if (desires[r] == 0) {
            remove_path(r);
            uint32_t counter = 0;
            if (!build(r, 0, counter)) {
                add_path(r);
            }
        }
    }
    PRINT(uint32_t p = cnt_completed * 100 / order.size();
          ASSERT(0 <= p && p <= 100, "invalid p: " + std::to_string(p));
          Printer() << "[EPIBT] progress: " << p << "%" << (p != 100 ? " bad\n" : "\n"););
}

double EPIBT::get_score() const {
    return cur_score;
}

std::vector<Action> EPIBT::get_actions() const {
    std::vector<Action> answer(robots.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        answer[r] = get_operations()[desires[r]][0];
        if (desires[r] == 0) {
            auto dist_r = get_hm().get(get_graph().get_to_node(robots[r].node, 1), robots[r].target);
            auto dist_c = get_hm().get(get_graph().get_to_node(robots[r].node, 2), robots[r].target);
            auto dist_w = get_hm().get(get_graph().get_to_node(robots[r].node, 3), robots[r].target);
            auto dist = std::min({dist_r, dist_c, dist_w});
            if (dist == dist_r) {
                answer[r] = Action::CR;
            } else if (dist == dist_c) {
                answer[r] = Action::CCR;
            } else {
                ASSERT(dist == dist_w, "invalid dist");
                answer[r] = Action::W;
            }
        }
    }
    return answer;
}
