#include <Objects/Environment/robot_handler.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/graph.hpp>
#include <Objects/Environment/heuristic_matrix.hpp>

bool Robot::is_disable() const {
    ASSERT((priority == -1) == !target, "invalid disable");
    return priority == -1;
}

RobotsHandler::RobotsHandler(uint32_t agents_num) : robots(agents_num) {
}

void RobotsHandler::update(const SharedEnvironment &env) {
    ASSERT(env.curr_states.size() == robots.size(), "invalid sizes");
    for (uint32_t r = 0; r < robots.size(); r++) {
        ASSERT(r < env.curr_states.size(), "invalid r");

        Position pos(env.curr_states[r].location + 1, env.curr_states[r].orientation);
        ASSERT(pos.is_valid(), "invalid position");
        uint32_t node = get_graph().get_node(pos);

        robots[r].node = node;
        robots[r].pos = pos.get_pos();
        robots[r].target = 0;
        robots[r].priority = -1;

        int task_id = env.curr_task_schedule[r];
        if (!env.task_pool.count(task_id)) {
            continue;
        }

        ASSERT(task_id != -1, "invalid task");

        auto &task = env.task_pool.at(task_id);
        uint32_t target = task.locations[task.idx_next_loc] + 1;

        ASSERT(target && target < get_map().get_size(), "invalid target");
        ASSERT(Position(target, 0).is_valid(), "invalid");

        uint32_t task_dist = 0;
        {
            uint32_t node = robots[r].node;
            for (int i = task.idx_next_loc; i < task.locations.size(); i++) {
                int to_pos = task.locations[i] + 1;
                task_dist += get_hm().get(node, to_pos);
                node = get_graph().get_node(Position(to_pos, 0));
            }
        }
        robots[r].target = target;
        robots[r].priority = task_dist;
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
