#include "pibt_star.hpp"

#include "../../Objects/assert.hpp"
#include "../../Objects/environment.hpp"
#include "../../Objects/guidance_graph.hpp"

void PIBTStar::check_for_no_exists(uint32_t r) const {
    for (uint32_t k = 0; k < PLANNER_DEPTH; k++) {
        for (uint32_t pos = 0; pos < get_env().get_size(); pos++) {
            ASSERT(map[k][pos] != r, "pizdec");
            ASSERT(map_gor[k][pos] != r, "pizdec");
            ASSERT(map_ver[k][pos] != r, "pizdec");
        }
    }
}

void PIBTStar::add_path_IMPL(uint32_t r) {
    ASSERT(r < robots.size(), "invalid r");
    check_for_no_exists(r);
    auto &robot = robots[r];
    ASSERT(robot.is_phantom, "is no phantom");
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

void PIBTStar::remove_path_IMPL(uint32_t r) {
    ASSERT(r < robots.size(), "invalid r");
    auto &robot = robots[r];
    ASSERT(!robot.is_phantom, "is phantom");
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
    robot.is_phantom = true;
    check_for_no_exists(r);
}

void PIBTStar::add_path(uint32_t r) {
    stack.push_back({r, robots[r].actions, true});
    add_path_IMPL(r);
}

void PIBTStar::remove_path(uint32_t r) {
    stack.push_back({r, robots[r].actions, false});
    remove_path_IMPL(r);
}

void PIBTStar::rollback() {
    auto [r, actions, is_add] = stack.back();
    stack.pop_back();
    if (is_add) {
        //remove_path();
    } else {
    }
}

void PIBTStar::rollback(uint32_t to_size) {
    while (stack.size() > to_size) {
        rollback();
    }
}

bool PIBTStar::build(uint32_t r) {
    ASSERT(0 <= r && r < robots.size(), "invalid r");
    auto &robot = robots[r];

    bool old_is_phantom = robot.is_phantom;
    if (!old_is_phantom) {
        remove_path(r);
    }

    check_for_no_exists(r);

    // A*
    struct Item {

        double score = 0;

        // время
        uint32_t t;

        // позиция
        Position p;

        // набранные действия
        Actions actions;

        // множество роботов на пути
        std::vector<uint32_t> set_robots;
    };

    struct comparator {
        bool operator()(const Item &lhs, const Item &rhs) const {
            return lhs.score < rhs.score;
        }
    };

    std::multiset<Item, comparator> S;
    S.insert({1.0 * get_env().get_dist(r, robots[r].p), 0, robots[r].p, get_w_actions(), {}});

    std::set<std::pair<uint32_t, Position>> visited;

    while (!S.empty()) {
        auto [score, t, p, actions, set_robots] = *S.begin();
        S.erase(S.begin());

        if (visited.count({t, p})) {
            continue;
        }
        visited.insert({t, p});

        if (t == PLANNER_DEPTH) {
            // finished

            check_for_no_exists(r);

            auto old_robots = robots;
            auto old_map = map;
            auto old_map_gor = map_gor;
            auto old_map_ver = map_ver;

            for (uint32_t other_r: set_robots) {
                ASSERT(r != other_r, "invalid r");
                remove_path(other_r);
            }

            robots[r].actions = actions;
            robots[r].is_done = true;
            add_path(r);

            bool ok = true;
            for (uint32_t other_r: set_robots) {
                if (!build(other_r)) {
                    ok = false;
                    break;
                }
            }

            if (ok) {
                return true;
            }

            // TODO: вернуть все как было
            robots = old_robots;
            map = old_map;
            map_gor = old_map_gor;
            map_ver = old_map_ver;

            check_for_no_exists(r);
            continue;
        }

        auto step = [&](Action action) {
            auto to = p.simulate_action(action);
            if (!to.is_valid()) {
                return;
            }
            if (visited.count({t + 1, to})) {
                return;
            }
            auto set = set_robots;

            auto add = [&](int r) {
                if (r != -1) {
                    if (robots[r].is_done) {
                        return false;// мы не сможем его сдвинуть
                    }
                    if (std::find(set.begin(), set.end(), r) == set.end()) {
                        set.push_back(r);
                    }
                }
                return true;
            };

            // там есть кто-то и он еще не построен
            if (!add(map[t][to.pos])) {
                return;
            }

            if (action == Action::FW) {
                uint32_t a = p.pos;
                uint32_t b = to.pos;
                if (a > b) {
                    std::swap(a, b);
                }
                if (b - a == 1) {
                    if (!add(map_gor[t][a])) {
                        return;
                    }
                } else {
                    if (!add(map_ver[t][a])) {
                        return;
                    }
                }
            }

            double to_score = score - get_env().get_dist(r, p) + get_env().get_dist(r, to);
            // TODO: add set size in score

            actions[t] = action;

            S.insert({to_score, t + 1, to, actions, set});
        };

        step(Action::FW);
        step(Action::CR);
        step(Action::CCR);
        step(Action::W);
    }

    check_for_no_exists(r);

    if (!old_is_phantom) {
        add_path(r);
    }
    return false;
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
        if (!robots[r].is_done) {
            build(r);
        }
    }

    std::vector<Action> actions(robots.size());
    for (uint32_t i = 0; i < robots.size(); i++) {
        ASSERT(!robots[i].is_phantom, "is phantom");
        actions[i] = robots[i].actions[0];
    }
    return actions;
}
