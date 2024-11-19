#ifdef MY_UNUSED

#include "pibt_star.hpp"

#include "Objects/Basic/assert.hpp"
#include "Objects/Basic/randomizer.hpp"
#include "Objects/Environment/environment.hpp"
#include "Objects/GuidanceGraph/guidance_graph.hpp"

void PIBTStar::add_path_IMPL(uint32_t r) {
    ASSERT(r < robots.size(), "invalid r");
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
}

void PIBTStar::set_path(uint32_t r, Actions actions) {
    stack.push_back({r, robots[r].actions, Operation::Type::SET});
    robots[r].actions = actions;
}

void PIBTStar::add_path(uint32_t r) {
    stack.push_back({r, robots[r].actions, Operation::Type::ADD});
    add_path_IMPL(r);
}

void PIBTStar::remove_path(uint32_t r) {
    stack.push_back({r, robots[r].actions, Operation::Type::REMOVE});
    remove_path_IMPL(r);
}

void PIBTStar::rollback() {
    auto [r, actions, type] = stack.back();
    stack.pop_back();
    if (type == Operation::Type::ADD) {
        ASSERT(robots[r].actions == actions, "no equal");
        remove_path_IMPL(r);
        robots[r].is_done = false;
    } else if (type == Operation::Type::REMOVE) {
        ASSERT(robots[r].actions == actions, "no equal");
        add_path_IMPL(r);
    } else if (type == Operation::Type::SET) {
        robots[r].actions = actions;
    } else {
        FAILED_ASSERT("unexpected type");
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

    uint32_t stack_top = stack.size();

    bool old_is_phantom = robot.is_phantom;
    if (!old_is_phantom) {
        remove_path(r);
    }

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

    std::set<std::pair<uint32_t, Actions>> visited;

    while (!S.empty()) {
        auto [score, t, p, actions, set_robots] = *S.begin();
        S.erase(S.begin());

        if (visited.count({t, actions})) {
            continue;
        }
        visited.insert({t, actions});

        if (t == PLANNER_DEPTH) {
            // finished

            uint32_t stack_top = stack.size();

            for (uint32_t other_r: set_robots) {
                ASSERT(r != other_r, "invalid r");
                remove_path(other_r);
            }

            set_path(r, actions);
            add_path(r);
            robots[r].is_done = true;

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

            rollback(stack_top);

            //ASSERT(robots == old_robots, "invalid robots rollback");
            /*for (uint32_t r = 0; r < robots.size(); r++) {
                ASSERT(robots[r].p == old_robots[r].p, "invalid p");
                ASSERT(robots[r].target == old_robots[r].target, "invalid target");
                ASSERT(robots[r].priority == old_robots[r].priority, "invalid priority");
                ASSERT(robots[r].is_phantom == old_robots[r].is_phantom, "invalid is_phantom");
                ASSERT(robots[r].is_done == old_robots[r].is_done, "invalid is_done");
                if (robots[r].actions != old_robots[r].actions) {
                    std::cout << "hello" << std::endl;
                }
                ASSERT(robots[r].actions == old_robots[r].actions, "invalid actions");
            }
            ASSERT(map == old_map, "invalid map rollback");
            ASSERT(map_gor == old_map_gor, "invalid map_gor rollback");
            ASSERT(map_ver == old_map_ver, "invalid map_ver rollback");*/
            // TODO: вернуть все как было
            /*robots = old_robots;
            map = old_map;
            map_gor = old_map_gor;
            map_ver = old_map_ver;*/

            continue;
        }

        auto step = [&](Action action) {
            auto to = p.simulate_action(action);
            if (!to.is_valid()) {
                return;
            }
            actions[t] = action;
            if (visited.count({t + 1, actions})) {
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

            int old_size = set.size();
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

            ASSERT(set.size() == old_size || set.size() == old_size + 1, "invalid size");

            if(set.size() > 2){
                return;
            }

            /*if (set.size() == old_size + 1) {
                uint32_t stack_top = stack.size();

                for (uint32_t other_r: set_robots) {
                    ASSERT(r != other_r, "invalid r");
                    remove_path(other_r);
                }

                set_path(r, actions);
                add_path(r);
                robots[r].is_done = true;

                bool ok = true;
                for (uint32_t other_r: set) {
                    if (!build(other_r)) {
                        ok = false;
                        break;
                    }
                }

                rollback(stack_top);

                if (!ok) {
                    return;
                }
            }*/

            double to_score = score - get_env().get_dist(r, p) + get_env().get_dist(r, to) + 1;
            // TODO: add set size in score
            //to_score += set.size() * 0.01;

            S.insert({to_score, t + 1, to, actions, set});
        };

        step(Action::FW);
        step(Action::CR);
        step(Action::CCR);
        step(Action::W);
    }

    if (!old_is_phantom) {
        rollback();
    }
    ASSERT(stack_top == stack.size(), "invalid stack size");

    /*for (uint32_t r = 0; r < robots.size(); r++) {
        ASSERT(robots[r].p == old_robots[r].p, "invalid p");
        ASSERT(robots[r].target == old_robots[r].target, "invalid target");
        ASSERT(robots[r].priority == old_robots[r].priority, "invalid priority");
        ASSERT(robots[r].is_phantom == old_robots[r].is_phantom, "invalid is_phantom");
        ASSERT(robots[r].is_done == old_robots[r].is_done, "invalid is_done");
        if (robots[r].actions != old_robots[r].actions) {
            std::cout << "hello" << std::endl;
        }
        ASSERT(robots[r].actions == old_robots[r].actions, "invalid actions");
    }
    ASSERT(map == old_map, "invalid map rollback");
    ASSERT(map_gor == old_map_gor, "invalid map_gor rollback");
    ASSERT(map_ver == old_map_ver, "invalid map_ver rollback");*/

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
    if (get_env().get_shared_env().curr_timestep == 500) {
        std::cout << "HELLO" << std::endl;
    }
    for (uint32_t r: order) {
        if (!robots[r].is_done) {
            build(r);
            stack.clear();
        }
    }

    std::vector<Action> actions(robots.size());
    int cnt_no_done = 0;
    for (uint32_t i = 0; i < robots.size(); i++) {
        cnt_no_done += !robots[i].is_done;
        ASSERT(!robots[i].is_phantom, "is phantom");
        actions[i] = robots[i].actions[0];
    }
    std::cout << "NO DONE: " << cnt_no_done << std::endl;
    return actions;
}

#endif