#include <Scheduler/scheduler.hpp>

#include <Scheduler/scheduler_solver.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>

#include <atomic>
#include <thread>
#include <unordered_map>
#include <unordered_set>

void MyScheduler::initialize(int preprocess_time_limit) {
}

std::vector<int> MyScheduler::plan(int time_limit, std::vector<int> &proposed_schedule) {
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

    std::cout << free_robots.size() << ' ' << free_tasks.size() << ' ' << env->new_freeagents.size() << ' '
              << env->new_tasks.size() << std::endl;

    if (free_robots.empty() || free_tasks.empty()) {
        return proposed_schedule;
    }

    // dp[r] = отсортированный вектор (dist, dist to start, task_id)
    static std::vector<std::vector<std::tuple<uint64_t, uint64_t, uint32_t>>> dp(env->num_of_agents);

    // для свободного робота будем поддерживать расстояния от него до всех задач
    // и будем постепенно обновлять это множество

    // returns (dist to start, total dist)
    auto get_dist = [&](uint32_t r, uint32_t t) -> std::pair<uint64_t, uint64_t> {
        std::array<uint64_t, 4> data{};
        constexpr static uint64_t MAX_SCORE = 1e15;
        uint64_t dist_to_start = MAX_SCORE;
        {
            uint32_t source = get_graph().get_node(
                    Position(env->curr_states[r].location, env->curr_states[r].orientation));

            ASSERT(env->task_pool[t].idx_next_loc == 0, "invalid idx next loc");

            uint32_t loc = env->task_pool[t].locations[0];
            for (uint32_t dir = 0; dir < 4; dir++) {
                Position p(loc, dir);
                uint32_t target = get_graph().get_node(p);
                data[dir] =//DefaultPlanner::get_h(env, env->curr_states[r].location, loc);
                        get_hm().get(source, target);
            }
            dist_to_start = *std::max_element(data.begin(), data.end());
        }
        /*for (uint32_t i = 0; i + 1 < env->task_pool[t].locations.size(); i++) {
            std::array<uint64_t, 4> new_data{MAX_SCORE, MAX_SCORE, MAX_SCORE, MAX_SCORE};
            for (uint32_t dir = 0; dir < 4; dir++) {
                Position source_pos(env->task_pool[t].locations[i], dir);
                uint32_t source_node = get_graph().get_node(source_pos);

                Position target_pos(env->task_pool[t].locations[i + 1], dir);
                uint32_t target_node = get_graph().get_node(target_pos);

                uint64_t dist =//DefaultPlanner::get_h(env, source_pos.get_pos(), target_pos.get_pos());
                        get_hm().get(source_node, target_node);
                new_data[dir] = std::min(new_data[dir], data[dir] + dist);
            }

            data = new_data;
        }*/
        for (uint32_t i = 0; i + 1 < env->task_pool[t].locations.size(); i++) {
            std::array<uint64_t, 4> new_data{MAX_SCORE, MAX_SCORE, MAX_SCORE, MAX_SCORE};
            for (uint32_t src_dir = 0; src_dir < 4; src_dir++) {
                for (uint32_t dst_dir = 0; dst_dir < 4; dst_dir++) {
                    Position source_pos(env->task_pool[t].locations[i], src_dir);
                    uint32_t source_node = get_graph().get_node(source_pos);

                    Position target_pos(env->task_pool[t].locations[i + 1], dst_dir);
                    uint32_t target_node = get_graph().get_node(target_pos);

                    uint64_t dist =//DefaultPlanner::get_h(env, source_pos.get_pos(), target_pos.get_pos());
                            get_hm().get(source_node, target_node);
                    new_data[dst_dir] = std::min(new_data[dst_dir], data[src_dir] + dist);
                }
            }
            data = new_data;
        }

        return {dist_to_start, *std::min_element(data.begin(), data.end())};
    };

    static std::vector<int> timestep_updated(free_robots.size(), -1);

    // обновляет множество расстояний
    auto rebuild = [&](uint32_t r) {
        dp[r].clear();

        for (uint32_t t: free_tasks) {
            auto [a, b] = get_dist(r, t);
            dp[r].emplace_back(b, a, t);
        }
        std::sort(dp[r].begin(), dp[r].end());
        timestep_updated[r] = env->curr_timestep;
    };

    {
        /*for (uint32_t r: free_robots) {
            rebuild(r);
        }*/

        std::vector<std::pair<int, uint32_t>> data;
        for (uint32_t r: free_robots) {
            data.emplace_back(timestep_updated[r], r);
        }
        std::sort(data.begin(), data.end());

        auto do_work = [&](uint32_t thr) {
            for (uint32_t i = thr; i < data.size(); i += THREADS) {
                if (std::chrono::steady_clock::now() >= end_time) {
                    break;
                }
                rebuild(data[i].second);
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

    std::unordered_set<uint32_t> used_tasks;
    for (uint32_t r: free_robots) {
        for (auto [dist, dist_to_start, task_id]: dp[r]) {
            if (!env->task_pool.count(task_id)) {
                continue;
            }
            if (env->task_pool[task_id].agent_assigned != -1) {
                continue;
            }
            if (used_tasks.count(task_id)) {
                continue;
            }

            used_tasks.insert(task_id);
            proposed_schedule[r] = task_id;
            if (dist_to_start <= 3) {
                done_proposed_schedule[r] = task_id;
            }
            break;
        }
    }

    /*std::atomic<bool> running = true;

    std::vector<std::tuple<uint64_t, uint64_t, uint32_t>> answers(THREADS);

    std::vector<std::atomic<uint32_t>> rs(THREADS);

    auto do_work = [&](uint32_t thr) {
        while (running) {
            uint32_t r = rs[thr];
            if (r == -1) {
                continue;
            }

            answers[thr] = {-1, -1, -1};

            uint32_t best_id = -1;
            uint64_t best_dist = -1;
            uint64_t best_start_dist = -1;
            for (uint32_t j = thr; j < free_tasks.size(); j += THREADS) {
                //if (std::chrono::steady_clock::now() >= end_time) {
                //break;
                //}
                uint32_t t = free_tasks[j];
                auto [start_dist, dist] = get_dist(r, t);
                if (dist < best_dist) {
                    best_start_dist = start_dist;
                    best_dist = dist;
                    best_id = j;
                }
            }
            answers[thr] = {best_dist, best_start_dist, best_id};
            rs[thr] = -1;
        }
    };

    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(do_work, thr);
    }

    auto calc = [&](uint32_t r) {
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            rs[thr] = r;
        }

        for (uint32_t thr = 0; thr < THREADS; thr++) {
            while (rs[thr] != -1) {}
        }

        return *std::min_element(answers.begin(), answers.end());
    };

    while (!free_robots.empty()) {
        if (std::chrono::steady_clock::now() >= end_time) {
            break;
        }

        uint32_t r = free_robots.back();

        auto [best_dist, best_start_dist, best_id] = calc(r);
        if (best_id == -1) {
            break;
        }

        uint32_t t = free_tasks[best_id];
        proposed_schedule[r] = t;

        //if (env->curr_states[r].location == env->task_pool[t].locations[0]) {
        if (best_start_dist <= 3) {
            done_proposed_schedule[r] = t;
        }

        free_robots.pop_back();

        std::swap(free_tasks[best_id], free_tasks.back());
        free_tasks.pop_back();
    }

    running = false;
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }*/

    //std::cout << free_robots.size() << ' ' << free_tasks.size() << std::endl;
    //std::cout << std::chrono::duration_cast<milliseconds>(end_time - std::chrono::steady_clock::now()).count() << "ms"
    //          << std::endl;
    return done_proposed_schedule;
}
