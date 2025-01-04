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

EPlanner::EPlanner() {
    env = new SharedEnvironment();
}

void EPlanner::initialize(int preprocess_time_limit) {}

void EPlanner::plan(int time_limit, std::vector<Action> &plan) {
    TimePoint end_time = env->plan_start_time + Milliseconds(time_limit - 50);
    Timer timer;

    plan.assign(env->num_of_agents, Action::W);

    get_robots_handler() = RobotsHandler(*env);

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

    // [depth] = { (score, time, plan) }
    std::vector<std::tuple<double, double, std::vector<Action>>> results(THREADS);

    auto do_work = [&](uint32_t thr, uint64_t seed) {
        Timer timer;
        PIBTS pibt(get_robots_handler().get_robots(), end_time, seed);
        pibt.simulate_pibt();
        double time = timer.get_ms();
        results[thr] = {pibt.get_score(), time, pibt.get_actions()};
    };

    static Randomizer rnd;
    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(do_work, thr, rnd.get());
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }

    double best_score = -1e300;
    std::sort(results.begin(), results.end(), std::greater<>());

    //Printer() << "RESULTS: ";
    for (const auto &[score, time, actions]: results) {
        //Printer() << "(" << score << ", " << time << ") ";
        if (best_score < score) {
            best_score = score;
            plan = actions;
        }
    }
    //Printer() << "\nbest: " << best_score << '\n';

    // (score, actions)
    /*std::vector<std::pair<double, std::vector<Action>>> results(THREADS);

    auto do_work = [&](uint32_t thr, uint64_t seed) {
        Randomizer rnd(seed);
        std::vector<PIBTS> pibts;
        for (uint32_t pibt_depth = thr; pibt_depth < 20 && get_now() < end_time; pibt_depth += THREADS) {
            Timer timer;
            PIBTS pibt(get_robots_handler().get_robots(), end_time, rnd.get());
            pibt.pibt_depth = pibt_depth;
            pibt.simulate_pibt();
            pibts.emplace_back(std::move(pibt));
        }
        if (pibts.empty()) {
            return;
        }

        //while (get_now() < end_time) {
        for(uint32_t step = 0; step < PIBTS_STEPS * pibts.size(); step++){
            auto& pibt = rnd.get(pibts);
            pibt.simulate_step();
        }

        // merge
        results[thr].first = -1e300;
        for (auto &pibt: pibts) {
            if (pibt.get_score() > results[thr].first) {
                results[thr] = {pibt.get_score(), pibt.get_actions()};
            }
        }
    };

    static Randomizer rnd;
    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(do_work, thr, rnd.get());
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }
    double best = -1e300;
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        if (best < results[thr].first) {
            best = results[thr].first;
            plan = results[thr].second;
        }
    }*/

#endif

#ifdef ENABLE_PRINT_LOG
    Printer() << "Planner: " << timer << '\n';
    if (timer.get_ms() > 300) {
        Printer() << "TIMEOUT\n";
    }
#endif
}
