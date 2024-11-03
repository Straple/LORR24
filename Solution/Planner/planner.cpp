#include "planner.hpp"

#include "../assert.hpp"
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
    std::vector<bool> map(env->map.size());
    ASSERT(env->map.size() == env->cols * env->rows, "invalid map size");
    for (int pos = 0; pos < map.size(); pos++) {
        map[pos] = env->map[pos] == 0;
    }
    PlannerSolver solver(env->rows, env->cols, map, {}, {});
    solver.build_dist();
}

// return next states for all agents
void EPlanner::plan(int time_limit, std::vector<Action> &plan) {
    plan.resize(env->num_of_agents, Action::W);

    TimePoint end_time = env->plan_start_time + std::chrono::milliseconds(time_limit - 60);

    constexpr uint32_t SOLVERS_SIZE = THREADS;
    std::vector<PlannerSolver> solvers;
    {
        std::vector<bool> map(env->map.size());
        ASSERT(env->map.size() == env->cols * env->rows, "invalid map size");
        for (int pos = 0; pos < map.size(); pos++) {
            map[pos] = env->map[pos] == 0;
        }
        std::vector<Position> robots_pos(env->num_of_agents);
        std::vector<int> robots_target(env->num_of_agents);
        for (int r = 0; r < robots_pos.size(); r++) {
            robots_pos[r] = Position(env->curr_states[r].location, env->curr_states[r].orientation, env);
            robots_target[r] = get_target(r);
        }
        solvers.assign(SOLVERS_SIZE, PlannerSolver(env->rows, env->cols, map, robots_pos, robots_target));
    }

    static Randomizer rnd(42);

    while (std::chrono::steady_clock::now() < end_time) {
        TimePoint end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);

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

    auto [solution_info, actions] = solvers[0].get();
    for (uint32_t r = 0; r < actions.size(); r++) {
        plan[r] = actions[r];
    }

    std::cout << "Total solution info: " //
              << solution_info.collision_count << ' ' //
              << solution_info.count_forward << ' '//
              << solution_info.sum_dist_change << std::endl;
}
