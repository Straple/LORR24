#include <Planner/pibt.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/time.hpp>
#include <Objects/Environment/environment.hpp>

#include <unordered_set>

bool PIBT::build(uint32_t r, int banned_desired, uint32_t depth, uint32_t &counter) {
    if (counter == -1 || (counter % 32 == 0 && get_now() > end_time)) {
        counter = -1;
        return false;
    }

    if (pos_to_robot[robots[r].pos] == r) {
        pos_to_robot.erase(robots[r].pos);
    }

    // (priority, dir)
    std::vector<std::pair<int64_t, int>> actions;
    for (int dir = 0; dir < 4; dir++) {
        if (dir == banned_desired) {
            continue;
        }
        Position to = get_graph().get_pos(robots[r].node);
        to = Position(to.get_x(), to.get_y(), dir);
        to = to.move_forward();
        if (!to.is_valid()) {
            continue;
        }
        // если там никого нет или он еще не посчитан
        if (!pos_to_robot.count(to.get_pos()) || desires[pos_to_robot.at(to.get_pos())] == -1) {

            uint32_t dist = get_hm().get(get_graph().get_node(to), robots[r].target);
            actions.emplace_back(dist, dir);
        }
    }

    std::sort(actions.begin(), actions.end());

    for (auto [_, dir]: actions) {
        Position to = get_graph().get_pos(robots[r].node);
        to = Position(to.get_x(), to.get_y(), dir);
        to = to.move_forward();

        if (!pos_to_robot.count(to.get_pos())) {
            // отлично! там никого нет
            pos_to_robot[to.get_pos()] = r;
            desires[r] = dir;
            return true;
        } else {
            // о нет! там кто-то есть

            if (counter > 10'000) {
                continue;
            }

            uint32_t to_r = pos_to_robot[to.get_pos()];
            pos_to_robot[to.get_pos()] = r;// теперь мы будем тут стоять
            desires[r] = dir;              // определим это направление

            // попробуем построить для to_r
            // и запретим ему ходить в нас (коллизия по ребру)
            if (build(to_r, (dir + 2) % 4, depth + 1, ++counter)) {
                // найс, получилось
                return true;
            }

            desires[r] = -1;
        }
    }

    pos_to_robot[robots[r].pos] = r;
    return false;
}

PIBT::PIBT(const std::vector<Robot> &robots, TimePoint end_time) : robots(robots), end_time(end_time), desires(robots.size(), -1), order(robots.size()) {
    for (uint32_t r = 0; r < robots.size(); r++) {
        pos_to_robot[robots[r].pos] = r;
    }

    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
        return robots[lhs].priority < robots[rhs].priority;
    });
}

void PIBT::solve() {
    for (uint32_t r: order) {
        if (std::chrono::steady_clock::now() > end_time) {
            break;
        }
        if (desires[r] == -1) {
            uint32_t counter = 0;
            build(r, -1, 0, counter);
        }
    }
}

[[nodiscard]] std::vector<Action> PIBT::get_actions() const {
    std::vector<Action> actions(robots.size(), Action::NA);

    std::unordered_set<uint32_t> used;
    // сначала разберемся с теми, кто поворачивается
    for (uint32_t r = 0; r < robots.size(); r++) {
        if (desires[r] == -1 || desires[r] == 4) {
            ASSERT(!used.count(get_graph().get_pos(robots[r].node).get_pos()), "already used");
            used.insert(get_graph().get_pos(robots[r].node).get_pos());
            actions[r] = Action::W;
        } else if (desires[r] != get_graph().get_pos(robots[r].node).get_dir()) {
            // нужно повернуться

            ASSERT(!used.count(get_graph().get_pos(robots[r].node).get_pos()), "already used");
            used.insert(get_graph().get_pos(robots[r].node).get_pos());

            auto calc = [&](Action rotate_type) {
                Position p = get_graph().get_pos(robots[r].node);
                int cnt = 0;
                while (p.get_dir() != desires[r]) {
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
        if (desires[r] == get_graph().get_pos(robots[r].node).get_dir()) {
            ASSERT(!used.count(get_graph().get_pos(robots[r].node).get_pos()), "already used");
            //used.insert(get_graph().get_pos(robots[r].node).get_pos());

            Position to = get_graph().get_pos(robots[r].node);
            to = to.move_forward();
            if (!used.count(to.get_pos())) {
                forwards[to.get_pos()].push_back(r);
            } else {
                forwards[robots[r].pos].push_back(r);
            }
        }
    }

    while (true) {
        std::vector<uint32_t> ids;
        for (auto &[pos, set]: forwards) {
            if (set.size() >= 2) {
                std::sort(set.begin(), set.end(), [&](uint32_t lhs, uint32_t rhs) {
                    int lhs_a = (get_graph().get_pos(robots[lhs].node).get_pos() == pos);
                    int rhs_a = (get_graph().get_pos(robots[rhs].node).get_pos() == pos);
                    if (lhs_a == rhs_a) {
                        return lhs < rhs;
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
            forwards[get_graph().get_pos(robots[r].node).get_pos()].push_back(r);
        }
    }

    for (auto &[pos, set]: forwards) {
        ASSERT(set.size() == 1, "invalid set");
        uint32_t r = set[0];
        if (get_graph().get_pos(robots[r].node).get_pos() == pos) {
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
