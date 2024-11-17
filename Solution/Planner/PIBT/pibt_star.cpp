#include "pibt_star.hpp"

#include "../../Objects/assert.hpp"
#include "../../Objects/environment.hpp"
#include "../../Objects/guidance_graph.hpp"

void PIBTStar::add_path(uint32_t r) {
    ASSERT(r < robots.size(), "invalid r");
    auto &robot = robots[r];
    ASSERT(robot.is_phantom == true, "is no phantom");
    Position p = robot.p;
    ASSERT(p.is_valid(), "invalid p");
    for (uint32_t k = 0; k < PLANNER_DEPTH; k++) {
        auto old = p;
        p = p.simulate_action(robot.actions[k]);
        ASSERT(p.is_valid(), "p is invalid");
        ASSERT(0 <= p.pos && p.pos < map[k].size(), "invalid pos: " + std::to_string(p.pos));
        ASSERT(map[k][p.pos] == -1, "invalid map: " + std::to_string(map[k][p.pos]));
        map[k][p.pos] = r;

        if (robot.actions[k] == Action::FW) {
            int pos = old.pos;
            int to = p.pos;
            if (pos > to) {
                std::swap(pos, to);
            }
            if (to - pos == 1) {
                // gor
                ASSERT(map_gor[k][pos] == -1, "invalid map");
                map_gor[k][pos] = r;
            } else {
                // ver
                ASSERT(to - pos == get_env().get_cols(), "invalid pos and to");
                ASSERT(map_ver[k][pos] == -1, "invalid map");
                map_ver[k][pos] = r;
            }
        }
    }
    robot.is_phantom = false;
}

void PIBTStar::remove_path(uint32_t r) {
    ASSERT(r < robots.size(), "invalid r");
    auto &robot = robots[r];
    ASSERT(robot.is_phantom == false, "is phantom");
    Position p = robot.p;
    ASSERT(p.is_valid(), "invalid p");
    for (uint32_t k = 0; k < PLANNER_DEPTH; k++) {
        auto old = p;
        p = p.simulate_action(robot.actions[k]);
        ASSERT(p.is_valid(), "p is invalid");
        ASSERT(0 <= p.pos && p.pos < map[k].size(), "invalid pos: " + std::to_string(p.pos));
        ASSERT(map[k][p.pos] == r, "invalid map: " + std::to_string(map[k][p.pos]));
        map[k][p.pos] = -1;

        if (robot.actions[k] == Action::FW) {
            int pos = old.pos;
            int to = p.pos;
            if (pos > to) {
                std::swap(pos, to);
            }
            if (to - pos == 1) {
                // gor
                ASSERT(map_gor[k][pos] == r, "invalid map");
                map_gor[k][pos] = -1;
            } else {
                // ver
                ASSERT(to - pos == get_env().get_cols(), "invalid pos and to");
                ASSERT(map_ver[k][pos] == r, "invalid map");
                map_ver[k][pos] = -1;
            }
        }
    }
}

bool PIBTStar::build(uint32_t r) {
}

PIBTStar::PIBTStar() {
    robots.resize(get_env().get_agents_size());
    for (uint32_t k = 0; k < PLANNER_DEPTH; k++) {
        map[k].assign(get_env().get_size(), -1);
        map_gor[k].resize(get_env().get_size(), -1);
        map_ver[k].resize(get_env().get_size(), -1);
    }
    for (uint32_t r = 0; r < robots.size(); r++) {
        robots[r].p = get_env().get_robot(r).p;
        robots[r].target = get_env().get_robot(r).target;
        robots[r].priority = get_env().get_robot(r).predicted_dist;
        add_path(r);
    }
}

std::vector<Action> PIBTStar::solve(const std::vector<uint32_t> &order) {
    for (uint32_t r: order) {
        if (robots[r].actions == get_w_actions()) {
            build(r);
        }
    }

    std::vector<Action> actions(robots.size());
    for (uint32_t i = 0; i < robots.size(); i++) {
        actions[i] = robots[i].actions[0];
    }
    return actions;
}
