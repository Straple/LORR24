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

const int INF = 1000000;

std::vector<int> Hungarian(const std::vector<std::vector<int>>& a, int n, int m) {
    vector<int> u (n+1), v (m+1), p (m+1), way (m+1);
    for (int i=1; i<=n; ++i) {
        p[0] = i;
        int j0 = 0;
        vector<int> minv (m+1, INF);
        vector<char> used (m+1, false);
        do {
            used[j0] = true;
            int i0 = p[j0],  delta = INF,  j1;
            for (int j=1; j<=m; ++j)
                if (!used[j]) {
                    int cur = a[i0][j]-u[i0]-v[j];
                    if (cur < minv[j])
                        minv[j] = cur,  way[j] = j0;
                    if (minv[j] < delta)
                        delta = minv[j],  j1 = j;
                }
            for (int j=0; j<=m; ++j)
                if (used[j])
                    u[p[j]] += delta,  v[j] -= delta;
                else
                    minv[j] -= delta;
            j0 = j1;
        } while (p[j0] != 0);
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }

    vector<int> ans (n+1, -1);
    for (int j=1; j<=m; ++j)
        ans[p[j]] = j;

    return ans;
}

std::vector<int> MyScheduler::GreedyShedule(int time_limit, std::vector<int> &proposed_schedule){
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
        // 12236
        return get_dist_to_start(r, t);

        // 11163
        uint32_t res = 0;
        uint32_t source = get_graph().get_node(Position(env->curr_states[r].location + 1, env->curr_states[r].orientation));
        for (int loc: env->task_pool[t].locations) {
            res += get_hm().get_to_pos(source, loc + 1);
            source = get_graph().get_node(Position(loc + 1, env->curr_states[r].orientation));
        }
        return res;
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

        for (uint32_t r: order) {
            for (auto [dist, task_id]: dp[r]) {
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

                ASSERT(env->task_pool.count(task_id), "no contains");
                ASSERT(env->task_pool[task_id].agent_assigned == -1, "already assigned");
                ASSERT(used_task_t[task_id] < launch_num, "already used");

                proposed_schedule[r] = task_id;
                used_task_t[task_id] = launch_num;
                if (get_dist_to_start(r, task_id) <= 3) {
                    done_proposed_schedule[r] = task_id;
                }
                break;
            }
        }

        /*std::vector<std::atomic<uint32_t>> thr_task(THREADS);
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
        }*/
    }

    //std::cout << "Scheduler: " << timer << std::endl;
    return done_proposed_schedule;
}
std::vector<int> MyScheduler::OptimizeShedule(int time_limit, std::vector<int>& schedule){

    auto get_dist_to_start = [&](uint32_t r, uint32_t t) {
        uint32_t source = get_graph().get_node(Position(env->curr_states[r].location + 1, env->curr_states[r].orientation));
        //ASSERT(env->task_pool[t].idx_next_loc == 0, "invalid idx next loc");
        uint32_t loc = env->task_pool[t].locations[0] + 1;
        return get_hm().get_to_pos(source, loc);
    };
    std::vector<int> done_proposed_schedule = schedule;

    TimePoint end_time = env->plan_start_time + std::chrono::milliseconds(time_limit);
    std::unordered_map<int,size_t> task_to_robot;

    int current_sum_dist = 0;
    for (size_t i = 0; i < schedule.size(); i++){
        if (schedule[i] >= 0){
            task_to_robot[schedule[i]] = i;
            current_sum_dist += get_dist_to_start(i, schedule[i]);  
        }
    }

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

    // RUN HUNGARY
    {
        int rb = min(free_robots.size(), free_tasks.size());
        // int rb =  min((int)free_robots.size(), 1);;
        std::vector<std::vector<int>> dist_matrix(rb+1, std::vector<int>(free_tasks.size() + 1, 0));
        for (int i = 0; i < rb; i++){
            for (int g = 0; g < free_tasks.size(); g++){
                dist_matrix[i+1][g+1] = get_dist_to_start(free_robots[i], free_tasks[g]);
                // dist_matrix[i+1][g+1] *= dist_matrix[i+1][g+1];
            }
        }
        auto ans = Hungarian(dist_matrix, rb, free_tasks.size());
        for (int i = 1; i < ans.size(); i++){
            std::cout << free_robots[i-1] << " -> " << free_tasks[ans[i]-1] << endl;
            if (ans[i] != -1){
                auto r = free_robots[i-1];
                auto t = free_tasks[ans[i]-1];
                schedule[r] = t;
                if (get_dist_to_start(r,t) <= 3){
                    done_proposed_schedule[r] = t;
                }
            }
        }
        return done_proposed_schedule;
    }

    if (free_robots.empty() || free_tasks.empty()){
        return done_proposed_schedule;
    }

    std::vector<int>closeset_task(free_robots.size(), -1);
    std::vector<int>distance_to_closest_task(free_robots.size(), -1);;
    for (size_t r_i = 0; r_i < free_robots.size(); r_i++){
        for (size_t t_i = 0; t_i < free_tasks.size(); t_i++){
            if (closeset_task[r_i] == -1 || distance_to_closest_task[r_i] > get_dist_to_start(free_robots[r_i], free_tasks[t_i])){
                closeset_task[r_i] = free_tasks[t_i];
                distance_to_closest_task[r_i] = get_dist_to_start(free_robots[r_i], free_tasks[t_i]);
            }
        }
    }

    // stores r_i
    std::vector<size_t> not_assigned_to_nearest;
    for (size_t r_i = 0; r_i < free_robots.size(); ++r_i){
        if (closeset_task[r_i] != schedule[free_robots[r_i]]){
            if (schedule[free_robots[r_i]] >= 0){
                not_assigned_to_nearest.push_back(r_i);
            }
        }
    }
    std::cout  << "not_assigned_to_nearest size: " << not_assigned_to_nearest.size() << " " << free_robots.size() << " " << free_tasks.size() << std::endl;

    for (const auto& flip_r_i: not_assigned_to_nearest){
        auto flip = free_robots[flip_r_i];
        assert(closeset_task[flip_r_i] != -1);
        auto task_to_flip_with = closeset_task[flip_r_i];
        if (task_to_robot.find(task_to_flip_with) == task_to_robot.end()) {
            std::cout << "TASK << " << task_to_flip_with << " IS EMPTY: SETTING " << flip << " for it " << endl;
            schedule[flip] = task_to_flip_with;
            continue;
        }
        assert(task_to_robot.find(task_to_flip_with) != task_to_robot.end());
        auto robot_to_flip_with = task_to_robot[task_to_flip_with];

        
        auto distance_was = get_dist_to_start(robot_to_flip_with, task_to_flip_with) + get_dist_to_start(flip, schedule[flip]);
        auto distance_new = get_dist_to_start(robot_to_flip_with, schedule[flip]) + get_dist_to_start(flip, task_to_flip_with);

        std::cout << "Distances: " <<  distance_was << " " << distance_new << std::endl;
        if (distance_was > distance_new) {
            std::cout << "PERFORM SWAP" << std::endl;
            std::swap(schedule[flip], schedule[robot_to_flip_with]);
        }
    }
    return done_proposed_schedule;

}

std::vector<int> MyScheduler::plan(int time_limit, std::vector<int> &proposed_schedule) {
    // auto shedule = GreedyShedule(time_limit/2, proposed_schedule);
    auto done_proposed_schedule = OptimizeShedule(time_limit, proposed_schedule);
    return done_proposed_schedule;
}



// std::vector<int> MyScheduler::planV2(int time_limit, std::vector<int> &proposed_schedule) {
//     Timer timer;

//     TimePoint end_time = env->plan_start_time + std::chrono::milliseconds(time_limit);
//     std::vector<uint32_t> free_robots, free_tasks;
//     for (uint32_t r = 0; r < env->num_of_agents; r++) {
//         uint32_t t = env->curr_task_schedule[r];
//         if (t == -1) {
//             free_robots.push_back(r);
//         }
//     }
//     for (auto &[t, task]: env->task_pool) {
//         if (task.agent_assigned == -1) {
//             free_tasks.push_back(t);
//         }
//     }

//     return proposed_schedule;
// }
