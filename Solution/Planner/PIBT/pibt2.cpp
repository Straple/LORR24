#include <Planner/PIBT/pibt2.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>

bool PIBT2::validate_path(uint32_t r) const {
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

bool PIBT2::check_path(uint32_t r) const {
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

std::array<uint32_t, PIBT2::DEPTH> PIBT2::get_path(uint32_t r) const {
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

uint32_t PIBT2::get_used(uint32_t r) const {
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
    // TODO: очень странно почему возможно встретить двух роботов
    //ASSERT(answer.size() <= 1, "invalid answer size");
    if (answer.size() > 1) {
        return -2;
    }
    if (answer.empty()) {
        return -1;
    }
    return *answer.begin();
}

void PIBT2::add_path(uint32_t r) {
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

        ASSERT(used_edge[depth][to_edge] == -1, "already used");
        used_edge[depth][to_edge] = r;

        ASSERT(used_pos[depth][to_pos] == -1, "already used");
        used_pos[depth][to_pos] = r;

        node = to_node;
    }
}

void PIBT2::remove_path(uint32_t r) {
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

bool PIBT2::build(uint32_t r) {

    uint32_t old_desired = robots[r].desired;

    // (priority, desired)
    std::vector<std::pair<int64_t, uint32_t>> steps;
    for (uint32_t desired = 1; desired < actions.size(); desired++) {
        robots[r].desired = desired;
        if (validate_path(r) && get_used(r) != -2) {
            auto path = get_path(r);

            int64_t priority = get_hm().get_to_pos(path.back(), get_robots_handler().get_robot(r).target);
            priority -= actions_weight[desired];
            steps.emplace_back(priority, desired);
        }
    }

    std::sort(steps.begin(), steps.end());

    for (auto [priority, desired]: steps) {
        robots[r].desired = desired;
        if (check_path(r)) {
            // отлично! там никого нет
            add_path(r);
            return true;
        } else {
            // о нет! там кто-то есть

            uint32_t to_r = get_used(r);
            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
            if (robots[to_r].desired != 0) {
                continue;
            }
            remove_path(to_r);
            add_path(r);

            if (build(to_r)) {
                return true;
            }

            remove_path(r);
            add_path(to_r);
        }
    }

    robots[r].desired = old_desired;
    return false;
}

PIBT2::PIBT2() {
    for (uint32_t depth = 0; depth < DEPTH; depth++) {
        used_pos[depth].resize(get_map().get_size(), -1);
        used_edge[depth].resize(get_graph().get_edges_size(), -1);
    }

    robots.resize(get_robots_handler().size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        robots[r].start_node = get_robots_handler().get_robot(r).node;
        add_path(r);
    }
}

std::vector<Action> PIBT2::solve(const std::vector<uint32_t> &order, const std::chrono::steady_clock::time_point end_time) {
    Timer timer;
    // PIBT
    for (uint32_t r: order) {
        if (std::chrono::steady_clock::now() > end_time) {
            break;
        }
        if (robots[r].desired == 0) {
            remove_path(r);
            if (!build(r)) {
                add_path(r);
            }
        }
    }

    std::vector<Action> answer(robots.size());

    for (uint32_t r = 0; r < robots.size(); r++) {
        answer[r] = actions[robots[r].desired][0];
    }

    std::cout << "PIBT2: " << timer << '\n';
    return answer;
}
