#include <Objects/Environment/robot_handler.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/graph.hpp>
#include <Objects/Environment/heuristic_matrix.hpp>

RobotsHandler::RobotsHandler(SharedEnvironment &env) {
    robots.resize(env.num_of_agents);
    for (uint32_t r = 0; r < robots.size(); r++) {
        robots[r].priority = 0;
        robots[r].node = 0;
        robots[r].target = 0;

        int task_id = env.curr_task_schedule[r];
        if (!env.task_pool.count(task_id)) {
            continue;
        }

        ASSERT(r < env.curr_states.size(), "invalid r");
        Position pos(env.curr_states[r].location, env.curr_states[r].orientation);
        ASSERT(pos.is_valid(),
               "invalid position: " + std::to_string(pos.get_x()) + " " + std::to_string(pos.get_y()) + " " +
                       std::to_string(pos.get_dir()));
        uint32_t node = get_graph().get_node(pos);

        robots[r].node = node;

        auto &task = env.task_pool.at(task_id);
        uint32_t target = task.get_next_loc();
        ASSERT(0 <= target && target < get_map().get_size(), "invalid target");
        ASSERT(Position(target, 0).is_valid(), "invalid");
        ASSERT(Position(target, 1).is_valid(), "invalid");
        ASSERT(Position(target, 2).is_valid(), "invalid");
        ASSERT(Position(target, 3).is_valid(), "invalid");
        uint32_t priority = get_hm().get_to_pos(node, target);
        robots[r].target = target;
        robots[r].priority = priority;
    }
}

const RobotsHandler::Robot &RobotsHandler::get_robot(uint32_t r) const {
    ASSERT(r < robots.size(), "invalid r");
    return robots[r];
}

uint32_t RobotsHandler::size() const {
    return robots.size();
}

RobotsHandler &get_robots_handler() {
    static RobotsHandler rh;
    return rh;
}
