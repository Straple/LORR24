#include <Objects/Environment/robot_handler.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/graph.hpp>
#include <Objects/Environment/heuristic_matrix.hpp>

RobotsHandler::RobotsHandler(SharedEnvironment &env) {
    robots.resize(env.num_of_agents);
    for (uint32_t r = 0; r < robots.size(); r++) {
        int task_id = env.curr_task_schedule[r];
        if (!env.task_pool.count(task_id)) {
            continue;
        }
        auto &task = env.task_pool.at(task_id);

        uint32_t node = get_graph().get_node(Position(env.curr_states[r].location, env.curr_states[r].orientation));
        uint32_t target = get_graph().get_node(Position(task.get_next_loc(), env.curr_states[r].orientation));
        uint32_t priority = get_hm().get(node, target);

        robots[r] = {node, target, priority};
    }
}

const RobotsHandler::Robot &RobotsHandler::get_robot(uint32_t r) const {
    ASSERT(r < robots.size(), "invalid r");
    return robots[r];
}

RobotsHandler &get_robots_handler() {
    static RobotsHandler rh;
    return rh;
}
