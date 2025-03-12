#include <Planner/eplanner.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>
#include <Planner/PIBT/epibt.hpp>
#include <Planner/PIBT/epibt_lns.hpp>
#include <Planner/PIBT/pibt.hpp>
#include <settings.hpp>

#include <algorithm>
#include <thread>

EPlanner::EPlanner(SharedEnvironment *env) : env(env) {}

EPlanner::EPlanner() {
    env = new SharedEnvironment();
}

std::vector<Action> EPlanner::plan(TimePoint end_time) {
    ETimer timer;
/*
    std::vector<Action> plan(env->num_of_agents, Action::W);


    // (score, actions, time, step)
    using ItemType = std::tuple<double, std::vector<Action>, int, int>;

    constexpr uint32_t THR = THREADS;
    std::vector<std::vector<ItemType>> results_pack(THR);

    PIBTS main_pibt_solver(get_robots_handler().get_robots(), end_time);

    auto do_work = [&](uint32_t thr, uint64_t seed) {
        ETimer timer;
        PIBTS pibt = main_pibt_solver;
        pibt.solve(seed);
        auto time = timer.get_ms();
        results_pack[thr].emplace_back(pibt.get_score(), pibt.get_actions(), time, pibt.get_step());
    };

    static Randomizer rnd;
    std::vector<std::thread> threads(THR);
    for (uint32_t thr = 0; thr < THR; thr++) {
        threads[thr] = std::thread(do_work, thr, rnd.get());
    }
    for (uint32_t thr = 0; thr < THR; thr++) {
        threads[thr].join();
    }

    std::vector<ItemType> results;
    for (uint32_t thr = 0; thr < THR; thr++) {
        for (auto &item: results_pack[thr]) {
            results.emplace_back(std::move(item));
        }
    }

    double best_score = -1e300;
    std::sort(results.begin(), results.end(), [&](const auto &lhs, const auto &rhs) {
        return std::get<0>(lhs) > std::get<0>(rhs);
    });

    PRINT(Printer() << "RESULTS(" << results.size() << "): ";);

    for (const auto &[score, actions, time, steps]: results) {
        PRINT(Printer() << "(" << score << ", " << time << ", " << steps << ") ";);
        if (best_score < score) {
            best_score = score;
            plan = actions;
        }
    }

    PRINT(Printer() << '\n';);
    PRINT(Printer() << "best: " << best_score << '\n';);
    PRINT(Printer() << "Planner: " << timer << '\n';);

    return plan;*/
    return {};
}
