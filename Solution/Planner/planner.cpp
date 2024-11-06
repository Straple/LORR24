#include "planner.hpp"

#include "../Objects/assert.hpp"
#include "../Objects/environment.hpp"
#include "global_dp.hpp"
#include "planner_solver.hpp"

int EPlanner::get_target(int r) const {
    int task_id = env->curr_task_schedule[r];
    if (task_id == -1) {
        return -1;
    }
    int t = 0;
    for (; t < env->task_pool.size() && task_id != env->task_pool[t].task_id; t++) {}
    ASSERT(t < env->task_pool.size() && env->task_pool[t].task_id == task_id, "invalid t");

    auto task = env->task_pool[t];
    int target = task.get_next_loc();
    ASSERT(0 <= target && target < env->cols * env->rows, "invalid target: " + std::to_string(target));

    return target;
}

EPlanner::EPlanner(SharedEnvironment *env) : env(env) {}

EPlanner::EPlanner() {
    env = new SharedEnvironment();
}

void EPlanner::initialize(int preprocess_time_limit) {
}

// return next states for all agents
void EPlanner::plan(int time_limit, std::vector<Action> &plan) {
    plan.assign(env->num_of_agents, Action::W);

    static std::ofstream output("log.txt");
    output << "timestep: " << env->curr_timestep << ' ';

    get_global_dp().init(env);

    TimePoint end_time = env->plan_start_time + std::chrono::milliseconds(time_limit - 30);

    auto start = std::chrono::steady_clock::now();
    auto robots_set = get_global_dp().split_robots(env);
    std::sort(robots_set.begin(), robots_set.end(), [&](const auto &lhs, const auto &rhs) {
        return lhs.size() > rhs.size();
    });
    output << std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - start).count() << "ms ";
    start = std::chrono::steady_clock::now();

    std::vector<PlannerSolver> solvers;
    std::set<int> used;
    for (const auto &set: robots_set) {
        std::vector<Position> robots_pos;
        std::vector<int> robots_target;
        for (int r: set) {
            ASSERT(!used.count(r), "already used");
            used.insert(r);
            robots_pos.emplace_back(env->curr_states[r].location, env->curr_states[r].orientation);
            robots_target.emplace_back(get_target(r));
        }
        solvers.emplace_back(robots_pos, robots_target);
    }
    ASSERT(used.size() == env->num_of_agents, "invalid used");

    output << std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - start).count() << "ms ";
    start = std::chrono::steady_clock::now();

    //std::cout << "build solvers time: " << std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - start).count() << "ms" << std::endl;

    static Randomizer rnd(36);

    std::vector<uint64_t> random_vals(solvers.size());
    for (uint32_t i = 0; i < solvers.size(); i++) {
        random_vals[i] = rnd.get();
    }

    auto do_work = [&](uint32_t thr) {
        uint64_t x = 0;
        for (int step = 0; step < 100'000; step++) {
            for (uint32_t i = thr; i < solvers.size(); i += THREADS) {
                //std::cout << thr << std::endl;
                TimePoint end_calc = std::chrono::steady_clock::now() + milliseconds(5);
                if (end_calc >= end_time) {
                    return;
                }
                solvers[i].run(end_calc, x + random_vals[i]);

                x = x * 13 + 7 + thr;
            }
        }
    };

    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < threads.size(); thr++) {
        threads[thr] = std::thread(do_work, thr);
    }
    for (uint32_t thr = 0; thr < threads.size(); thr++) {
        threads[thr].join();
    }

    output << std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - start).count() << "ms ";
    start = std::chrono::steady_clock::now();

    /*while (std::chrono::steady_clock::now() < end_time) {
        //TimePoint end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);

        std::vector<uint64_t> random_vals(solvers.size());
        for (uint32_t i = 0; i < solvers.size(); i++) {
            random_vals[i] = rnd.get();
        }

        auto do_work = [&](uint32_t thr) {
            for (uint32_t i = thr; i < solvers.size(); i += THREADS) {
                solvers[i].run(end_time, random_vals[i]);
            }
        };
        std::vector<std::thread> threads(THREADS);
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            threads[thr] = std::thread(do_work, thr);
        }
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            threads[thr].join();
        }

        // merge answers
        std::sort(solvers.begin(), solvers.end(), [&](const auto &lhs, const auto &rhs) {
            return lhs.get_x(lhs.get_solution_info()) > rhs.get_x(rhs.get_solution_info());
        });

        if (solvers.size() > 3) {
            solvers.resize(3);
            while (solvers.size() < SOLVERS_SIZE) {
                solvers.emplace_back(solvers[rnd.get(0, 2)]);
            }
        }
    }

    std::cout << solvers[0].get_trivial_solution_info() << std::endl;
    std::cout << solvers[0].get_solution_info() << std::endl;
    ASSERT(solvers[0].get_trivial_solution_info() == solvers[0].get_solution_info(), "invalid solutions");*/

    SolutionInfo total_info;
    for (int idx = 0; idx < solvers.size(); idx++) {
        auto [solution_info, actions] = solvers[idx].get();
        //std::cout << "info: " << solution_info << std::endl;
        //std::cout << "expe: " << solvers[idx].get_trivial_solution_info() << std::endl;
        //ASSERT(solvers[idx].get_trivial_solution_info() == solution_info, "failed");
        if (solution_info.collision_count == 0) {
            total_info = total_info + solution_info;
            //std::cout << actions.size() << ' ' << robots_set[idx].size() << std::endl;
            ASSERT(actions.size() == robots_set[idx].size(), "unmatch sizes");
            for (int i = 0; i < actions.size(); i++) {
                plan[robots_set[idx][i]] = actions[i];
            }
        }
    }

    output << std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - start).count() << "ms ";
    start = std::chrono::steady_clock::now();
    output << "\n";

    //std::cout << step << ' ' << total_info << ", time: " << std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() << "ms" << std::endl;

    output << total_info << '\n';
    output << robots_set.size() << ": ";
    for (auto &vec: robots_set) {
        output << vec.size() << " ";
    }
    output << std::endl;
}
