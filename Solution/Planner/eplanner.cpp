#include <Planner/eplanner.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>
#include <Planner/PIBT/pibt.hpp>
#include <Planner/PIBT/pibt2.hpp>
#include <Planner/PIBT/pibt3.hpp>
#include <Planner/PIBT/pibts.hpp>
#include <settings.hpp>

#include <algorithm>
#include <thread>

EPlanner::EPlanner(SharedEnvironment *env) : env(env) {}

EPlanner::EPlanner() {
    env = new SharedEnvironment();
}

void EPlanner::plan(int time_limit, std::vector<Action> &plan) {
    TimePoint end_time = env->plan_start_time + Milliseconds(time_limit - 20);
    Timer timer;

    plan.assign(env->num_of_agents, Action::W);

#ifdef ENABLE_PIBT
    /*std::vector<uint32_t> order(env->num_of_agents);
    iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
        return get_robots_handler().get_robot(lhs).priority < get_robots_handler().get_robot(rhs).priority;
    });

    //std::vector<std::unordered_map<uint32_t, uint32_t>> weights(get_robots_handler().size());
    PIBT2 pibt(get_robots_handler().get_robots()//, weights
    );

    //PIBTS pibt(get_robots_handler().get_robots());
    plan = pibt.solve(order, end_time);*/

    //PIBTS pibt(get_robots_handler().get_robots());
    //plan = pibt.solve(end_time, 0);

    /*std::vector<std::pair<int64_t, std::vector<Action>>> results;
    for (uint32_t pibt_depth = 5; pibt_depth < 20 && get_now() < end_time; pibt_depth++) {
        PIBTS pibt(get_robots_handler().get_robots());
        pibt.pibt_depth = pibt_depth;
        uint64_t seed = 0;

        auto kek = pibt.solve(end_time, seed);
        results.emplace_back(pibt.get_score(), kek);
    }

    Printer() << "PIBTS:";
    for (auto [score, plan]: results) {
        Printer() << ' ' << score;
    }
    Printer() << '\n';

    std::sort(results.begin(), results.end(), std::greater<>());
    plan = results[0].second;*/

    /*std::vector<std::pair<double, std::vector<Action>>> results(THREADS);

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
#ifdef ENABLE_PRINT_LOG
    Printer() << "PIBTS: ";
#endif
    for (uint32_t thr = 0; thr < THREADS; thr++) {
#ifdef ENABLE_PRINT_LOG
        Printer() << results[thr].first << ' ';
#endif
        if (best < results[thr].first) {
            best = results[thr].first;
            plan = results[thr].second;
        }
    }
#ifdef ENABLE_PRINT_LOG
    Printer() << '\n';
#endif*/

    /*static Randomizer rnd(228);
    PIBTS pibt(get_robots_handler().get_robots(), end_time, rnd.get());
    pibt.simulate_pibt();
    plan = pibt.get_actions();*/

    // [thr] = { (score, time, plan) }
    std::vector<std::tuple<double, double, std::vector<Action>>> results(THREADS);

    auto do_work = [&](uint32_t thr, uint64_t seed) {
        Timer timer;
        PIBTS pibt(get_robots_handler().get_robots(), end_time, seed);
        pibt.simulate_pibt();
        double time = timer.get_ms();
        results[thr] = {pibt.get_score(), time, pibt.get_actions()};
    };

    static Randomizer rnd(228);
    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(do_work, thr, rnd.get());
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }

    double best_score = -1e300;
    std::sort(results.begin(), results.end(), std::greater<>());

#ifdef ENABLE_PRINT_LOG
    Printer() << "RESULTS: ";
#endif
    for (const auto &[score, time, actions]: results) {
#ifdef ENABLE_PRINT_LOG
        Printer() << "(" << score << ", " << time << ") ";
#endif
        if (best_score < score) {
            best_score = score;
            plan = actions;
        }
    }
#ifdef ENABLE_PRINT_LOG
    Printer() << "\nbest: " << best_score << '\n';
#endif



#endif

#ifdef ENABLE_PRINT_LOG
    Printer() << "Planner: " << timer << '\n';
    if (timer.get_ms() > 300) {
        Printer() << "TIMEOUT\n";
    }
#endif
}
