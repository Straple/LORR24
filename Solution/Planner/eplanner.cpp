#include "eplanner.hpp"

#include "Objects/Basic/assert.hpp"
#include "Objects/Environment/environment.hpp"
#include "PIBT/pibt.hpp"
#include "PIBT/pibt_solver.hpp"
#include "PIBT/pibt_star.hpp"
#include "Solver/global_dp.hpp"
#include "Solver/planner_solver.hpp"

#include <algorithm>
#include <thread>

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
    static TimePoint start = std::chrono::steady_clock::now();
    TimePoint end_time = env->plan_start_time + std::chrono::milliseconds(time_limit - 30);

    plan.assign(env->num_of_agents, Action::W);
    get_env().build_robots();

    get_env().build_robot_dists(std::min(std::chrono::steady_clock::now() + std::chrono::milliseconds(UPDATE_DYNAMICS_DIST_MATRIX_TIME), end_time));

#ifdef ENABLE_PIBT_SOLVER
    PIBTSolver pibt_solver;
    plan = pibt_solver.solve(end_time);
#endif

#ifdef ENABLE_PIBT
    std::vector<uint32_t> order(get_env().get_agents_size());
    iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
        return get_env().get_robot(lhs).predicted_dist < get_env().get_robot(rhs).predicted_dist;
    });

    PIBT pibt;
    plan = pibt.solve(order, end_time);
#endif

#ifdef ENABLE_PIBT_STAR
    std::vector<uint32_t> order(get_env().get_agents_size());
    iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
        return get_env().get_robot(lhs).predicted_dist < get_env().get_robot(rhs).predicted_dist;
    });

    PIBTStar pibt_star;
    plan = pibt_star.solve(order);
#endif

#ifdef ENABLE_PLANNER_MACHINE
    if (planner_machine == nullptr) {
        planner_machine = new PlannerMachine();
    }
    planner_machine->run(end_time);
    planner_machine->set_plan(plan);
    planner_machine->simulate_world();
#endif

#ifdef ENABLE_PLANNER_SOLVER
    get_global_dp().init(env);
    std::vector<int> robot_target(env->num_of_agents, -1);
    for (auto &task: env->task_pool) {
        if (task.agent_assigned != -1) {
            robot_target[task.agent_assigned] = task.get_next_loc();
            ASSERT(0 <= robot_target[task.agent_assigned] && robot_target[task.agent_assigned] < env->cols * env->rows,
                   "invalid target: " + std::to_string(robot_target[task.agent_assigned]));
        }
    }

    auto start = std::chrono::steady_clock::now();
    auto robots_set = get_env().split_robots(env);
    std::sort(robots_set.begin(), robots_set.end(), [&](const auto &lhs, const auto &rhs) {
        return lhs.size() > rhs.size();
    });
    //output << std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - start).count() << "ms ";
    start = std::chrono::steady_clock::now();

    std::vector<PlannerSolver> solvers;
    for (const auto &set: robots_set) {
        std::vector<Position> robots_pos;
        std::vector<int> robots_target;
        for (int r: set) {
            robots_pos.emplace_back(env->curr_states[r].location, env->curr_states[r].orientation);
            robots_target.emplace_back(robot_target[r]);
        }
        solvers.emplace_back(robots_pos, robots_target);
    }

    //output << std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - start).count() << "ms ";
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
                TimePoint end_calc = std::chrono::steady_clock::now() + milliseconds(2);
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

    //output << std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - start).count() << "ms ";
    start = std::chrono::steady_clock::now();

    SolutionInfo total_info;
    for (int idx = 0; idx < solvers.size(); idx++) {
        auto [solution_info, actions] = solvers[idx].get();
        //std::cout << "info: " << solution_info << std::endl;
        //std::cout << "expe: " << solvers[idx].get_trivial_solution_info() << std::endl;
        //ASSERT(solvers[idx].get_trivial_solution_info() == solution_info, "failed");
        if (solution_info.collision_count[0] == 0) {
            total_info = total_info + solution_info;
            //std::cout << actions.size() << ' ' << robots_set[idx].size() << std::endl;
            ASSERT(actions.size() == robots_set[idx].size(), "unmatch sizes");
            for (int i = 0; i < actions.size(); i++) {
                plan[robots_set[idx][i]] = actions[i];
            }
        }
    }
#endif

    //static std::ofstream output("planner_log.txt");
    //output << "planner time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() << "ms" << '\n';
    std::cout << "time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() << "ms" << '\n';
}
