#include "eplanner.hpp"

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>
#include <Planner/PIBT/pibt.hpp>
#include <Planner/PIBT/pibt2.hpp>
#include <settings.hpp>

#include <algorithm>
#include <thread>

EPlanner::EPlanner(SharedEnvironment *env) : env(env) {}

EPlanner::EPlanner() { env = new SharedEnvironment(); }

void EPlanner::initialize(int preprocess_time_limit) {}

// return next states for all agents
void EPlanner::plan(int time_limit, std::vector<Action> &plan) {
    static TimePoint start = std::chrono::steady_clock::now();
    TimePoint end_time = env->plan_start_time + std::chrono::milliseconds(time_limit - 50);

    plan.assign(env->num_of_agents, Action::W);

    get_robots_handler() = RobotsHandler(*env);

#ifdef ENABLE_PIBT
    std::vector<uint32_t> order(env->num_of_agents);
    iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
        return get_robots_handler().get_robot(lhs).priority < get_robots_handler().get_robot(rhs).priority;
    });

    PIBT2 pibt;
    plan = pibt.solve(order, end_time);
#endif
}
