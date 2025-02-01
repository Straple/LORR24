#include <Planner/eplanner.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>
#include <Planner/PIBT/pibt.hpp>
#include <Planner/PIBT/pibt2.hpp>
#include <Planner/PIBT/pibts.hpp>
#include <Planner/PIBT/pmps.hpp>
#include <settings.hpp>

#include <algorithm>
#include <thread>

EPlanner::EPlanner(SharedEnvironment *env) : env(env) {}

EPlanner::EPlanner() {
    env = new SharedEnvironment();
}

void EPlanner::plan(int time_limit, std::vector<Action> &plan) {
    TimePoint end_time = env->plan_start_time + Milliseconds(time_limit - 50);
    Timer timer;

    plan.assign(env->num_of_agents, Action::W);

#ifdef ENABLE_PIBTS
    {
        // (score, actions)
        using ItemType = std::tuple<double, std::vector<Action>
#ifdef ENABLE_PRINT_LOG
                // time, desires, changes, step, log_str
                , int, std::vector<uint32_t>, std::vector<int64_t>, uint32_t, std::string
#endif
        >;

        constexpr uint32_t THR = THREADS;
        std::vector<std::vector<ItemType>> results_pack(THR);

        auto do_work = [&](uint32_t thr, uint64_t seed) {
            Timer timer;
            PIBTS pibt(get_robots_handler().get_robots(), end_time, seed);
            pibt.simulate_pibt();
            auto time = timer.get_ms();
            results_pack[thr].emplace_back(pibt.get_score(), pibt.get_actions()
#ifdef ENABLE_PRINT_LOG
                    , time, pibt.get_desires(), pibt.get_changes(), pibt.step, pibt.log.str()
#endif
            );
        };

        static Randomizer rnd(228);
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

#ifdef ENABLE_PRINT_LOG
        Printer() << "RESULTS(" << results.size() << "): ";
#endif
        for (const auto &[score, actions
#ifdef ENABLE_PRINT_LOG
                    , time, desires, changes, steps, log_str
#endif
            ]: results) {
#ifdef ENABLE_PRINT_LOG
            Printer() << "(" << score << ", " << time << ", " << steps << ") ";
#endif
            if (best_score < score) {
                best_score = score;
                plan = actions;
            }
        }
#ifdef ENABLE_PRINT_LOG
        Printer() << "\nbest: " << best_score << '\n';
        Printer() << "best log:\n" << std::get<6>(results[0]) << '\n';
        Printer() << "worst log:\n" << std::get<6>(results.back()) << '\n';
#endif


#ifdef ENABLE_PRINT_LOG
        static std::vector<uint32_t> total_desires(get_operations().size());
        static std::vector<int64_t> total_changes(get_operations().size());
        for (uint32_t r = 0; r < env->num_of_agents; r++) {
            if (results.empty()) {
                break;
            }
            uint32_t desired = std::get<3>(results[0])[r];
            int64_t change = std::get<4>(results[0])[r];
            ASSERT(0 <= desired && desired < get_operations().size(), "invalid desired");
            total_desires[desired]++;
            total_changes[desired] += change;
        }
        Printer() << "Desires:\n";
        for (uint32_t d = 0; d < total_desires.size(); d++) {
            Printer() << d << ' ' << get_operations()[d] << ' ' << total_desires[d] << ' ' << total_changes[d] << '\n';
        }
#endif

    }
#endif

#ifdef ENABLE_PMPS

    {
        constexpr uint32_t THR = 8;
        // (score, actions)
        using ItemType = std::tuple<double, std::vector<Action>>;
        std::vector<ItemType> results(THR);

        auto do_work = [&](uint32_t thr, uint64_t seed) {
            PMPS solver(get_robots_handler().get_robots(), end_time, env->curr_timestep);
            solver.solve(seed);
            results[thr] = {solver.get_score(), solver.get_actions()};
        };

        static Randomizer rnd(228);
        std::vector<std::thread> threads(THR);
        for (uint32_t thr = 0; thr < THR; thr++) {
            threads[thr] = std::thread(do_work, thr, rnd.get());
        }
        for (uint32_t thr = 0; thr < THR; thr++) {
            threads[thr].join();
        }
        std::sort(results.begin(), results.end(), [&](const auto &lhs, const auto &rhs) {
            return std::get<0>(lhs) > std::get<0>(rhs);
        });
        plan = std::get<1>(results[0]);

#ifdef ENABLE_PRINT_LOG
        Printer() << "RESULTS(" << results.size() << "): ";
#endif
        for (const auto &[score, actions]: results) {
#ifdef ENABLE_PRINT_LOG
            Printer() << score << ' ';
#endif
        }
#ifdef ENABLE_PRINT_LOG
        Printer() << '\n';
#endif
    }

#endif

#ifdef ENABLE_PRINT_LOG
    Printer() << "Planner: " << timer << '\n';
    //if (timer.get_ms() > 300) {
    //    Printer() << "TIMEOUT\n";
    //}
#endif
}
