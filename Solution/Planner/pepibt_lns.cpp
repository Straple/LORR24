#include <Planner/pepibt_lns.hpp>

#include <Planner/epibt_lns.hpp>

#include <thread>

PEPIBT_LNS::PEPIBT_LNS(const std::vector<Robot> &robots, TimePoint end_time) : robots(robots), end_time(end_time), actions(robots.size(), Action::W) {
}

void PEPIBT_LNS::solve(uint64_t seed) {
    EPIBT_LNS main_pibt(get_robots_handler().get_robots(), end_time);

    // (score, actions, time, step)
    using ItemType = std::tuple<double, std::vector<Action>, int, int>;

    constexpr uint32_t THR = THREADS;
    std::vector<std::vector<ItemType>> results_pack(THR);

    auto do_work = [&](uint32_t thr, uint64_t seed) {
        ETimer timer;
        EPIBT_LNS pibt = main_pibt;
        pibt.solve(seed);
        auto time = timer.get_ms();
        results_pack[thr].emplace_back(pibt.get_score(), pibt.get_actions(), time, pibt.get_step());
    };

    Randomizer rnd(seed);
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

    PRINT(Printer() << "[PEPIBT_LNS] results: ";);
    for (const auto &[score, plan, time, steps]: results) {
        PRINT(Printer() << "(" << score << ", " << time << ", " << std::min(steps, (int) plan.size()) << "+" << std::max(0, steps - (int) plan.size()) << ") ";);
        if (best_score < score) {
            best_score = score;
            actions = plan;
        }
    }
    PRINT(Printer() << '\n';);
}

std::vector<Action> PEPIBT_LNS::get_actions() const {
    return actions;
}
