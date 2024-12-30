#include <Planner/PIBT/pibt_lns.hpp>

#include <Objects/Environment/environment.hpp>

PIBT_LNS::PIBT_LNS(std::vector<Robot> robots) : robots(robots), weights(robots.size()) {
}

std::pair<double, std::array<std::vector<Action>, PIBT_LNS_DEPTH>> PIBT_LNS::simulate(TimePoint end_time) const {
    std::array<std::vector<Action>, PIBT_LNS_DEPTH> result;
    auto cur_robots = robots;
    double score = 0;
    for (uint32_t depth = 0; depth < PIBT_LNS_DEPTH; depth++) {
        std::vector<uint32_t> order(robots.size());
        iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
            return cur_robots[lhs].priority < cur_robots[rhs].priority;
        });

        // call PIBT
        PIBT2 pibt(cur_robots, weights);
        auto actions = pibt.solve(order, end_time);

        // update result
        result[depth] = actions;
        for (uint32_t r = 0; r < robots.size(); r++) {
            int32_t old_dist = get_hm().get_to_pos(cur_robots[r].node, cur_robots[r].target);
            int32_t new_dist = get_hm().get_to_pos(get_graph().get_to_node(cur_robots[r].node, actions[r]), cur_robots[r].target);
            // в идеале old_dist = new_dist + 1
            // old_dist - new_dist <= 1
            //score += -(old_dist - new_dist - 1);
        }

        // simulate move
        for (uint32_t r = 0; r < robots.size(); r++) {
            cur_robots[r].node = get_graph().get_to_node(cur_robots[r].node, actions[r]);
            cur_robots[r].priority = get_hm().get_to_pos(cur_robots[r].node, cur_robots[r].target);
        }
    }
    for (uint32_t r = 0; r < robots.size(); r++) {
        int32_t old_dist = get_hm().get_to_pos(robots[r].node, cur_robots[r].target);
        int32_t new_dist = get_hm().get_to_pos(cur_robots[r].node, cur_robots[r].target);
        score += old_dist - new_dist;
        //score += (2 - r * 1.0 / robots.size()) * get_hm().get_to_pos(cur_robots[r].node, cur_robots[r].target) * 1.0 / std::max(1U, get_hm().get_to_pos(robots[r].node, cur_robots[r].target));
    }
    return {score, result};
}

void PIBT_LNS::update_weights(const std::array<std::vector<Action>, PIBT_LNS_DEPTH> &actions) {
    for (uint32_t r = 0; r < robots.size(); r++) {
        Robot robot = robots[r];
        std::array<uint32_t, PIBT_LNS_DEPTH> edges_path{};
        for (uint32_t depth = 0; depth < PIBT_LNS_DEPTH; depth++) {
            uint32_t to_node = get_graph().get_to_node(robot.node, actions[depth][r]);
            uint32_t to_edge = get_graph().get_to_edge(robot.node, actions[depth][r]);
            edges_path[depth] = to_edge;

            int32_t old_dist = get_hm().get_to_pos(robot.node, robot.target);
            int32_t new_dist = get_hm().get_to_pos(to_node, robot.target);
            //score += old_dist - new_dist;
            int32_t m = old_dist - new_dist;
            if (m <= 0) {
                m--;
                // m < 0
                m = -m;
                // m > 0
                for (uint32_t i = 0; i <= depth; i++) {
                    weights[r][edges_path[i]] += m;
                }
            }

            // simulate move
            robot.node = to_node;
            robot.priority = get_hm().get_to_pos(robot.node, robot.target);
        }
    }
}

std::vector<Action> PIBT_LNS::solve(TimePoint end_time) {
    std::vector<Action> result(robots.size(), Action::W);
    double result_score = -1e300;
    std::cout << "PIBT_LNS: ";
    while (get_now() < end_time) {
        auto [score, actions] = simulate(end_time);
        std::cout << score << ' ';
        if (score > result_score) {
            result_score = score;
            result = actions[0];
        }

        //weights.assign(robots.size(), {});
        update_weights(actions);
    }
    std::cout << std::endl;
    return result;
}

// 3118 -> 3139 -> 3169