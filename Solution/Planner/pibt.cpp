#include "pibt.hpp"

#include "../Objects/assert.hpp"
#include "../Objects/environment.hpp"

bool PIBT::build(uint32_t r, int priority) {
    pos_to_robot.erase(robots[r].p.pos);

    // (priority, dir)
    std::vector<std::pair<int, int>> actions;
    for (int dir = 0; dir < 4; dir++) {
        Position to = robots[r].p;
        to.dir = dir;
        to = to.move_forward();
        if (to.is_valid()) {
            // если там никого нет или он еще не посчитан
            if (!pos_to_robot.count(to.pos) || robots[pos_to_robot.at(to.pos)].dir == -1) {
                actions.emplace_back(get_env().get_dist(robots[r].p, to.pos) + get_env().get_dist(to, robots[r].target), dir);
            }
        }
    }

    std::sort(actions.begin(), actions.end());

    for (auto [_, dir]: actions) {
        Position to = robots[r].p;
        to.dir = dir;
        to = to.move_forward();

        if (!pos_to_robot.count(to.pos)){
            // отлично! там никого нет
            pos_to_robot[to.pos] = r;
            robots[r].dir = dir;
            return true;
        }
        else{
            // о нет! там кто-то есть
            robots[r].dir = dir;



            robots[r].dir = -1;
        }
    }

    pos_to_robot[robots[r].p.pos] = r;
    return false;
}

PIBT::PIBT(const std::vector<Position> &robots_pos, const std::vector<int> &robots_target) {
    ASSERT(robots_pos.size() == robots_target.size() && !robots_pos.empty(), "invalid sizes");

    robots.resize(robots_pos.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        robots[r].p = robots_pos[r];
        robots[r].target = robots_target[r];
        pos_to_robot[robots[r].p.pos] = r;
    }
}

std::vector<Action> PIBT::solve() {
    for (uint32_t r = 0; r < robots.size(); r++) {
    }
}