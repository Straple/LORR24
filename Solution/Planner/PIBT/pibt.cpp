#include "pibt.hpp"

#include "Objects/Basic/assert.hpp"
#include "Objects/Environment/environment.hpp"
#include "Objects/GuidanceGraph/guidance_graph.hpp"

#include <unordered_set>

bool PIBT::build(uint32_t r, int banned_direction) {
    if (pos_to_robot[robots[r].p.pos] == r) {
        pos_to_robot.erase(robots[r].p.pos);
    }

    // (priority, dir)
    std::vector<std::pair<int64_t, int>> actions;
    for (int dir = 0; dir < 4; dir++) {
        if (dir == banned_direction) {
            continue;
        }
        Position to = robots[r].p;
        to.dir = dir;
        to = to.move_forward();
        if (to.is_valid()) {
            // если там никого нет или он еще не посчитан
            if (!pos_to_robot.count(to.pos) || robots[pos_to_robot[to.pos]].dir == -1) {
                actions.emplace_back(get_env().get_dist(robots[r].p, to.pos) +
                                             //get_env().get_dist(to, robots[r].target)
                                     get_env().get_dist(r, to)
                                     ,
                                     dir);
            }
        }
    }

    std::sort(actions.begin(), actions.end());

    for (auto [_, dir]: actions) {
        Position to = robots[r].p;
        to.dir = dir;
        to = to.move_forward();

        if (!pos_to_robot.count(to.pos)) {
            // отлично! там никого нет
            pos_to_robot[to.pos] = r;
            robots[r].dir = dir;
            return true;
        } else {
            // о нет! там кто-то есть

            uint32_t to_r = pos_to_robot[to.pos];
            pos_to_robot[to.pos] = r;// теперь мы будем тут стоять
            robots[r].dir = dir;     // определим это направление

            // попробуем построить для to_r
            // и запретим ему ходить в нас (коллизия по ребру)
            if (build(to_r, (dir + 2) % 4)) {
                // найс, получилось
                return true;
            }

            robots[r].dir = -1;
        }
    }

    pos_to_robot[robots[r].p.pos] = r;
    return false;
}

PIBT::PIBT() {
    robots.resize(get_env().get_agents_size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        robots[r].p = get_env().get_robot(r).p;
        robots[r].target = get_env().get_robot(r).target;
        robots[r].priority = get_env().get_robot(r).predicted_dist;
        pos_to_robot[robots[r].p.pos] = r;
    }
}

std::vector<Action> PIBT::solve(const std::vector<uint32_t> &order, const std::chrono::steady_clock::time_point end_time) {
    for (uint32_t r: order) {
        if (std::chrono::steady_clock::now() > end_time) {
            break;
        }
        if (robots[r].dir == -1) {
            build(r);
        }
    }

    std::vector<Action> actions(robots.size(), Action::NA);
    std::unordered_set<uint32_t> used;
    // сначала разберемся с теми, кто поворачивается
    for (uint32_t r = 0; r < robots.size(); r++) {
        if (robots[r].dir == -1 || robots[r].dir == 4) {
            ASSERT(!used.count(robots[r].p.pos), "already used");
            used.insert(robots[r].p.pos);
            actions[r] = Action::W;
            continue;
        }
        if (robots[r].dir != robots[r].p.dir) {
            // нужно повернуться

            ASSERT(!used.count(robots[r].p.pos), "already used");
            used.insert(robots[r].p.pos);

            auto calc = [&](Action rotate_type) {
                Position p = robots[r].p;
                int cnt = 0;
                while (p.dir != robots[r].dir) {
                    cnt++;
                    p = p.simulate_action(rotate_type);
                }
                return cnt;
            };
            if (calc(Action::CR) < calc(Action::CCR)) {
                actions[r] = Action::CR;
            } else {
                actions[r] = Action::CCR;
            }
        }
    }

    std::unordered_map<uint32_t, std::vector<uint32_t>> forwards;
    for (uint32_t r = 0; r < robots.size(); r++) {
        if (robots[r].dir == robots[r].p.dir) {
            ASSERT(!used.count(robots[r].p.pos), "already used");
            Position to = robots[r].p;
            to = to.move_forward();
            if (!used.count(to.pos)) {
                forwards[to.pos].push_back(r);
            } else {
                forwards[robots[r].p.pos].push_back(r);
            }
        }
    }

    while (true) {
        std::vector<uint32_t> ids;
        for (auto &[pos, set]: forwards) {
            if (set.size() >= 2) {
                std::sort(set.begin(), set.end(), [&](uint32_t lhs, uint32_t rhs) {
                    int lhs_a = (robots[lhs].p.pos == pos);
                    int rhs_a = (robots[rhs].p.pos == pos);
                    if (lhs_a == rhs_a) {
                        return robots[lhs].priority < robots[rhs].priority;
                    } else {
                        return lhs_a > rhs_a;
                    }
                });
                while (set.size() >= 2) {
                    ids.push_back(set.back());
                    set.pop_back();
                }
            }
        }
        if (ids.empty()) {
            break;
        }
        for (uint32_t r: ids) {
            forwards[robots[r].p.pos].push_back(r);
        }
    }

    for (auto &[pos, set]: forwards) {
        ASSERT(set.size() == 1, "invalid set");
        uint32_t r = set[0];
        if (robots[r].p.pos == pos) {
            actions[r] = Action::W;
        } else {
            actions[r] = Action::FW;
        }
    }

    for (uint32_t r = 0; r < robots.size(); r++) {
        ASSERT(actions[r] != Action::NA, "NA action");
    }

    return actions;
}

double PIBT::get_score() const {
    double res = 0;
    std::vector<uint32_t> order(robots.size());
    iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
        return robots[lhs].priority < robots[rhs].priority;
    });
    for (uint32_t i = 0; i < robots.size(); i++) {
        uint32_t r = order[i];
        if (robots[r].dir == -1) {
            continue;
        }

        // (priority, dir)
        std::vector<std::pair<int64_t, int>> actions;
        for (int dir = 0; dir < 4; dir++) {
            Position to = robots[r].p;
            to.dir = dir;
            to = to.move_forward();
            if (to.is_valid()) {
                actions.emplace_back(get_env().get_dist(robots[r].p, to.pos) + get_env().get_dist(r, to), dir);
            }
        }

        std::sort(actions.begin(), actions.end());
        int k = 0;
        for (uint32_t i = 0; i < actions.size(); i++) {
            if (actions[i].second == robots[r].dir) {
                k = i;
                break;
            }
        }

        res += (actions.size() - k) * 1.0 * ((int64_t) robots.size() * robots.size() - (int64_t) i * i) * 1.0 / robots.size();
    }
    return res;
}
