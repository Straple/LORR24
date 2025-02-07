#include <Objects/Environment/robot_handler.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/graph.hpp>
#include <Objects/Environment/heuristic_matrix.hpp>
#include <Objects/Environment/dynamic_heuristic_matrix.hpp>

RobotsHandler::RobotsHandler(uint32_t agents_num) : robots(agents_num) {
}

void RobotsHandler::update(SharedEnvironment &env) {
    ASSERT(env.curr_states.size() == robots.size(), "invalid sizes");
    for (uint32_t r = 0; r < robots.size(); r++) {
        ASSERT(r < env.curr_states.size(), "invalid r");

        Position pos(env.curr_states[r].location + 1, env.curr_states[r].orientation);
        ASSERT(pos.is_valid(),
               "invalid position: " + std::to_string(pos.get_x()) + " " + std::to_string(pos.get_y()) + " " +
               std::to_string(pos.get_dir()));
        uint32_t node = get_graph().get_node(pos);

        robots[r].prev_node = robots[r].node;
        robots[r].node = node;
        robots[r].prev_target = robots[r].target;
        robots[r].target = 0;
        robots[r].priority = 0;

        int task_id = env.curr_task_schedule[r];
        if (!env.task_pool.count(task_id)) {
            continue;
        }

        auto &task = env.task_pool.at(task_id);
        uint32_t target = task.get_next_loc() + 1;

        ASSERT(0 <= target && target < get_map().get_size(), "invalid target");
        ASSERT(Position(target, 0).is_valid(), "invalid");
        ASSERT(Position(target, 1).is_valid(), "invalid");
        ASSERT(Position(target, 2).is_valid(), "invalid");
        ASSERT(Position(target, 3).is_valid(), "invalid");

        uint32_t task_dist = 0;
        {
            uint32_t node = robots[r].node;
            for (int i = task.idx_next_loc; i < task.locations.size(); i++) {
                int to_pos = task.locations[i] + 1;
                if (i == 0) {
                    task_dist += get_hm().get(node, to_pos) * get_hm().get(node, to_pos);
                } else {
                    task_dist += get_hm().get(node, to_pos);
                }
                node = get_graph().get_node(Position(to_pos, 0));
            }
        }
        // TODO:
        // if task_dist + env.curr_timestep > get_test_info().steps_num
        // then this robot is free
        robots[r].target = target;
        robots[r].priority = task_dist;
    }
    /*for (uint32_t r = 0; r < robots.size(); r++) {
        if (!robots[r].prev_node) {
            continue;
        }
        if (robots[r].target != robots[r].prev_target) {
            robots[r].penalty = 1;
        } else if (get_hm().get(robots[r].prev_node, robots[r].target) <=
                   get_hm().get(robots[r].node, robots[r].target)) {
            robots[r].penalty++;
            //robots[r].penalty /= 1.2;
        } else {
            robots[r].penalty--;
            //robots[r].penalty *= 1.2;
        }
    }*/
    double max_priority = -1e300;
    double min_priority = 1e300;
    for (uint32_t r = 0; r < robots.size(); r++) {
        //robots[r].priority += robots[r].penalty * 0.01;

        max_priority = std::max(max_priority, robots[r].priority);
        min_priority = std::min(min_priority, robots[r].priority);
    }
    for (uint32_t r = 0; r < robots.size(); r++) {
        robots[r].priority = 1 - (robots[r].priority - min_priority) / (max_priority - min_priority);
    }
}

const Robot &RobotsHandler::get_robot(uint32_t r) const {
    ASSERT(r < robots.size(), "invalid r");
    return robots[r];
}

const std::vector<Robot> &RobotsHandler::get_robots() const {
    return robots;
}

uint32_t RobotsHandler::size() const {
    return robots.size();
}

RobotsHandler &get_robots_handler() {
    static RobotsHandler rh;
    return rh;
}
