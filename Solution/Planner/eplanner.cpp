#include <Planner/eplanner.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>
#include <Planner/PIBT/pibt.hpp>
#include <Planner/PIBT/pibt2.hpp>
#include <Planner/PIBT/pibt3.hpp>
#include <Planner/PIBT/pibt_lns.hpp>
#include <Planner/PIBT/pibts.hpp>
#include <settings.hpp>

#include <algorithm>
#include <thread>

EPlanner::EPlanner(SharedEnvironment *env) : env(env) {}

EPlanner::EPlanner() { env = new SharedEnvironment(); }

void EPlanner::initialize(int preprocess_time_limit) {}

void EPlanner::plan(int time_limit, std::vector<Action> &plan) {
    static TimePoint start = std::chrono::steady_clock::now();
    TimePoint end_time = env->plan_start_time + std::chrono::milliseconds(time_limit - 10);

    plan.assign(env->num_of_agents, Action::W);

    get_robots_handler() = RobotsHandler(*env);

#ifdef ENABLE_PIBT
    std::vector<uint32_t> order(env->num_of_agents);
    iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
        return get_robots_handler().get_robot(lhs).priority < get_robots_handler().get_robot(rhs).priority;
    });

    //std::vector<std::unordered_map<uint32_t, uint32_t>> weights(get_robots_handler().size());
    //PIBT2 pibt(get_robots_handler().get_robots(), weights);

    PIBTS pibt(get_robots_handler().get_robots());
    plan = pibt.solve(end_time);

    //PIBT_LNS pibt(get_robots_handler().get_robots());
    //plan = pibt.solve(end_time);
#endif
}
