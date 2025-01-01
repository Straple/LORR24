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
    TimePoint end_time = env->plan_start_time + std::chrono::milliseconds(time_limit - 50);

    plan.assign(env->num_of_agents, Action::W);

    get_robots_handler() = RobotsHandler(*env);

#ifdef ENABLE_PIBT
    /*std::vector<uint32_t> order(env->num_of_agents);
    iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
        return get_robots_handler().get_robot(lhs).priority < get_robots_handler().get_robot(rhs).priority;
    });*/

    //std::vector<std::unordered_map<uint32_t, uint32_t>> weights(get_robots_handler().size());
    //PIBT2 pibt(get_robots_handler().get_robots(), weights);

    //PIBTS pibt(get_robots_handler().get_robots());
    //plan = pibt.solve(end_time);

    std::vector<std::pair<int64_t, std::vector<Action>>> results(THREADS);

    auto do_work = [&](uint32_t thr, uint64_t seed) {
        PIBTS pibt(get_robots_handler().get_robots());
        results[thr].second = pibt.solve(end_time, seed);
        results[thr].first = pibt.get_score();
    };

    static Randomizer rnd;
    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(do_work, thr, rnd.get());
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }

    int64_t best = -1e18;
    //std::cout << "PIBTS: ";
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        //std::cout << results[thr].first << ' ';
        if (best < results[thr].first) {
            best = results[thr].first;
            plan = results[thr].second;
        }
    }
    //std::cout << '\n';
    //PIBT_LNS pibt(get_robots_handler().get_robots());

#endif
}
