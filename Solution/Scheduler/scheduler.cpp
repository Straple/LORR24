#include <Scheduler/scheduler.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/time.hpp>
#include <Objects/Environment/environment.hpp>

#include <atomic>
#include <thread>
#include <unordered_map>
#include <unordered_set>

void MyScheduler::initialize(int preprocess_time_limit) {
}

std::vector<int> MyScheduler::plan(int time_limit, std::vector<int> &proposed_schedule) {
    static uint32_t launch_num = 0;
    launch_num++;

    Timer timer;

    TimePoint end_time = env->plan_start_time + std::chrono::milliseconds(time_limit);

    std::vector<uint32_t> free_robots, free_tasks;
    for (uint32_t r = 0; r < env->num_of_agents; r++) {
        uint32_t t = env->curr_task_schedule[r];
        if (t == -1) {
            free_robots.push_back(r);
        }
    }
    for (auto &[t, task]: env->task_pool) {
        if (task.agent_assigned == -1) {
            free_tasks.push_back(t);
        }
    }

    //std::cout << free_robots.size() << ' ' << free_tasks.size() << ' ' << env->new_freeagents.size() << ' '
    //          << env->new_tasks.size() << std::endl;

    if (free_robots.empty() || free_tasks.empty()) {
        return proposed_schedule;
    }

    // dp[r] = отсортированный вектор (dist, task_id)
    static std::vector<std::vector<std::pair<uint32_t, uint32_t>>> dp(env->num_of_agents);

    // для свободного робота будем поддерживать расстояния от него до всех задач
    // и будем постепенно обновлять это множество

    constexpr static uint32_t MAX_SCORE = -1;

    auto get_dist_to_start = [&](uint32_t r, uint32_t t) {
        uint32_t source = get_graph().get_node(Position(env->curr_states[r].location + 1, env->curr_states[r].orientation));
        ASSERT(env->task_pool[t].idx_next_loc == 0, "invalid idx next loc");
        uint32_t loc = env->task_pool[t].locations[0] + 1;
        return get_hm().get_to_pos(source, loc);
    };

    auto get_dist = [&](uint32_t r, uint32_t t) {
        return get_dist_to_start(r, t);
    };

    static std::vector<int> timestep_updated(free_robots.size(), -1);

    // обновляет множество расстояний
    auto rebuild = [&](uint32_t r) {
        dp[r].clear();

        for (uint32_t t: free_tasks) {
            dp[r].emplace_back(get_dist(r, t), t);
        }
        std::sort(dp[r].begin(), dp[r].end());
        timestep_updated[r] = env->curr_timestep;
    };

    std::vector<uint32_t> order = free_robots;
    std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
        return timestep_updated[lhs] < timestep_updated[rhs];
    });

    {
        auto do_work = [&](uint32_t thr) {
            for (uint32_t i = thr; i < order.size(); i += THREADS) {
                if (get_now() >= end_time) {
                    break;
                }
                rebuild(order[i]);
            }
        };

        std::vector<std::thread> threads(THREADS);
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            threads[thr] = std::thread(do_work, thr);
        }
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            threads[thr].join();
        }
    }

    auto done_proposed_schedule = proposed_schedule;
    {
        static std::vector<uint32_t> used_task_t(500'000);// max task available

        std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
            return timestep_updated[lhs] > timestep_updated[rhs];
        });

        std::vector<std::atomic<uint32_t>> thr_task(THREADS);
        std::vector<std::atomic<uint32_t>> answer(THREADS);

        auto do_work = [&](uint32_t thr) {
            while (true) {
                uint32_t r = thr_task[thr];
                if (r == -2) {
                    break;
                }
                if (r == -1) {
                    continue;
                }
                for (uint32_t i = thr; i < dp[r].size(); i += THREADS) {
                    auto [dist, task_id] = dp[r][i];
                    // not used in this timestep
                    if (used_task_t[task_id] == launch_num) {
                        continue;
                    }
                    // this task is available
                    if (!env->task_pool.count(task_id)) {
                        continue;
                    }
                    // robot already used this task
                    if (env->task_pool[task_id].agent_assigned != -1) {
                        continue;
                    }

                    answer[thr] = i;
                    break;
                }

                thr_task[thr] = -1;
            }
        };

        std::vector<std::thread> threads(THREADS);
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            thr_task[thr] = -1;
            threads[thr] = std::thread(do_work, thr);
        }
        for (uint32_t r: order) {
            for (uint32_t thr = 0; thr < THREADS; thr++) {
                answer[thr] = -1;
                thr_task[thr] = r;
            }
            for (uint32_t thr = 0; thr < THREADS; thr++) {
                while (thr_task[thr] != -1) {}
            }

            uint32_t index = *std::min_element(answer.begin(), answer.end());
            if (index == -1) {
                break;
            }
            ASSERT(index < dp[r].size(), "invalid index");
            auto [dist, task_id] = dp[r][index];

            ASSERT(env->task_pool.count(task_id), "no contains");
            ASSERT(env->task_pool[task_id].agent_assigned == -1, "already assigned");
            //ASSERT(!used_tasks.count(task_id), "already used");
            ASSERT(used_task_t[task_id] < launch_num, "already used");

            //used_tasks.insert(task_id);
            proposed_schedule[r] = task_id;
            used_task_t[task_id] = launch_num;
            if (get_dist_to_start(r, task_id) <= 3) {
                done_proposed_schedule[r] = task_id;
            }
        }
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            thr_task[thr] = -2;
        }
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            threads[thr].join();
        }
    }

    std::cout << "Scheduler: " << timer << std::endl;
    return done_proposed_schedule;
}
