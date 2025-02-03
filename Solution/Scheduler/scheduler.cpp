#include <Scheduler/scheduler.hpp>
#include <Scheduler/journey_graph.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/time.hpp>
#include <Objects/Environment/environment.hpp>
#include <Scheduler/hungarian.hpp>
#include <settings.hpp>

#include <atomic>
#include <thread>
#include <unordered_map>
#include <unordered_set>


MyScheduler::MyScheduler(SharedEnvironment *env) : env(env), solver(env) {

}

int get_dist_to_start(uint32_t r, uint32_t t, SharedEnvironment *env) {
    uint32_t source = get_graph().get_node(env->curr_states[r].location + 1, env->curr_states[r].orientation);
    ASSERT(env->task_pool[t].idx_next_loc == 0, "invalid idx next loc");
    uint32_t loc = env->task_pool[t].locations[0] + 1;
    return get_hm().get(source, loc);
}

int get_dist(uint32_t r, uint32_t t, SharedEnvironment *env) {
    uint32_t dist = 0;
    uint32_t source = get_graph().get_node(Position(env->curr_states[r].location + 1, env->curr_states[r].orientation));
    for (int i = 0; i < env->task_pool[t].locations.size(); i++) {
        int loc = env->task_pool[t].locations[i];
        if (i == 0) {
            dist += get_dhm().get(source, loc + 1);
        } else {
            dist += get_hm().get(source, loc + 1);
        }
        source = get_graph().get_node(Position(loc + 1, env->curr_states[r].orientation));
    }
    return dist;
}

const int INF = 1000000;

void MyScheduler::solver_schedule(int time_limit, std::vector<int> &proposed_schedule) {
    TimePoint point = get_now();
    solver.update();
#ifndef ENABLE_TRIVIAL_SCHEDULER
    solver.rebuild_dp(point + Milliseconds(SCHEDULER_REBUILD_DP_TIME));
#endif
    solver.triv_solve(point + Milliseconds(SCHEDULER_REBUILD_DP_TIME + SCHEDULER_TRIV_SOLVE_TIME));
    //solver.solve(get_now() + Milliseconds(150));
    proposed_schedule = solver.get_schedule();
}

void MyScheduler::greedy_schedule(int time_limit, std::vector<int> &proposed_schedule) {
    static uint32_t launch_num = 0;
    launch_num++;

    Timer timer;

    TimePoint end_time = get_now() + std::chrono::milliseconds(time_limit);

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
        return;
    }

    // dp[r] = отсортированный вектор (dist, task_id)
    static std::vector<std::vector<std::pair<uint32_t, uint32_t>>> dp(env->num_of_agents);

    // для свободного робота будем поддерживать расстояния от него до всех задач
    // и будем постепенно обновлять это множество

    static std::vector<int> timestep_updated(free_robots.size(), -1);

    // обновляет множество расстояний
    auto rebuild = [&](uint32_t r) {
        dp[r].clear();

        for (uint32_t t: free_tasks) {
            dp[r].emplace_back(get_dist(r, t, env), t);
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

#ifdef ENABLE_PRINT_LOG
    Printer() << "free robots: " << free_robots.size() << '\n';
    Printer() << "free tasks: " << free_tasks.size() << '\n';
#endif

    double workload = env->num_of_agents * 1.0 / get_map().get_count_free();
    uint32_t done_weight = 5;
    if (workload > 0.4) {
        done_weight = 1;
    }

    {
        static std::vector<uint32_t> used_task_t(500'000);// max task available

        // (dist, r, index)
        std::priority_queue<std::tuple<uint32_t, uint32_t, uint32_t>, std::vector<std::tuple<uint32_t, uint32_t, uint32_t>>, std::greater<>> Heap;
        for (uint32_t r: free_robots) {
            if (!dp[r].empty()) {
                Heap.push({dp[r][0].first, r, 0});
            }
        }

        while (!Heap.empty()) {
            auto [dist, r, index] = Heap.top();
            Heap.pop();

            uint32_t task_id = dp[r][index].second;
            ASSERT(dist == dp[r][index].first, "invalid dist");

            // not used in this timestep
            if (used_task_t[task_id] == launch_num
                // this task is available
                || !env->task_pool.count(task_id)
                // robot already used this task
                || env->task_pool[task_id].agent_assigned != -1) {

                if (index + 1 < dp[r].size()) {
                    Heap.push({dp[r][index + 1].first, r, index + 1});
                }

                continue;
            }

            ASSERT(env->task_pool.count(task_id), "no contains");
            ASSERT(env->task_pool[task_id].agent_assigned == -1, "already assigned");
            ASSERT(used_task_t[task_id] < launch_num, "already used");

            proposed_schedule[r] = task_id;
            used_task_t[task_id] = launch_num;
        }
    }
}

int calc_full_distance(Task &task) {
    /*if (task.full_distance != -1){
        return;
    }*/
    auto &hm = get_hm();
    int dist_sum = 0;
    for (size_t i = 0; i + 1 < task.locations.size(); ++i) {
        uint32_t from = task.locations[i] + 1;
        uint32_t to = task.locations[i + 1] + 1;
        ASSERT(false, "from must be graph node (INCORRECT), to must be map node (CORRECT)");
        dist_sum += hm.get(from, to);
    }
    return dist_sum;
    //task.full_distance = dist_sum;
}

// const int INF = 1000000;

// std::vector<int> Hungarian(const std::vector<std::vector<int>> &a, int n, int m) {
//     vector<int> u(n + 1), v(m + 1), p(m + 1), way(m + 1);
//     for (int i = 1; i <= n; ++i) {
//         p[0] = i;
//         int j0 = 0;
//         vector<int> minv(m + 1, INF);
//         vector<char> used(m + 1, false);
//         do {
//             used[j0] = true;
//             int i0 = p[j0], delta = INF, j1;
//             for (int j = 1; j <= m; ++j)
//                 if (!used[j]) {
//                     int cur = a[i0][j] - u[i0] - v[j];
//                     if (cur < minv[j])
//                         minv[j] = cur, way[j] = j0;
//                     if (minv[j] < delta)
//                         delta = minv[j], j1 = j;
//                 }
//             for (int j = 0; j <= m; ++j)
//                 if (used[j])
//                     u[p[j]] += delta, v[j] -= delta;
//                 else
//                     minv[j] -= delta;
//             j0 = j1;
//         } while (p[j0] != 0);
//         do {
//             int j1 = way[j0];
//             p[j0] = p[j1];
//             j0 = j1;
//         } while (j0);
//     }

//     vector<int> ans(n + 1, -1);
//     for (int j = 1; j <= m; ++j)
//         ans[p[j]] = j;

//     return ans;
// }


// std::vector<int> MyScheduler::artem_schedule(int time_limit, std::vector<int> &schedule) {

//     Timer timer;

//     for (size_t i = 0; i < schedule.size(); i++){
//         schedule[i] = -1;
//     }
//     auto& jg = get_jg();
//     for (const auto task_id: env->new_tasks){
//         jg.AddTask(task_id, env->task_pool[task_id]);
//     }
//     for (const auto robot_id: env->new_freeagents){
//         jg.Complete(robot_id);
//     }

//     jg.ProccessOptimizationQ(100);

//     TimePoint end_time = get_now() + std::chrono::milliseconds(time_limit);

//     struct Entity {
//         uint32_t cordinate;
//         size_t index;
//         bool is_robot;

//         bool operator<(const Entity &other) const {
//             return std::tie(cordinate, index, is_robot) < std::tie(other.cordinate, other.index, other.is_robot);
//         }
//     };

//     std::vector<Entity> xs;
//     std::vector<Entity> ys;


//     std::vector<size_t> free_robots = jg.GetFreeRobots();
//     std::vector<size_t> free_tasks = jg.GetAvailableStarts();

 
//     // if (true) {
//     //     std::cout << "IM HERE" << std::endl;
//     // }

//     size_t fr_robots = 0;
//     for (uint32_t r = 0; r < env->num_of_agents; r++) {
//         uint32_t t = env->curr_task_schedule[r];
//         if (t == -1) {
//             fr_robots++;
//         }
//     }
//     size_t fr_tasks = 0;
//     for (auto &[t, task]: env->task_pool) {
//         if (task.agent_assigned == -1) {
//             fr_tasks++;
//         }
//     }

//     for (size_t i = 0; i < free_robots.size(); ++i){
//         const auto r = free_robots[i];
//             assert(env->curr_task_schedule[r] == -1);
//             auto pos = Position(env->curr_states[r].location + 1, env->curr_states[r].orientation);
//             xs.push_back({Position(env->curr_states[r].location + 1, env->curr_states[r].orientation).get_x(),
//                             i, true});
//             ys.push_back({Position(env->curr_states[r].location + 1, env->curr_states[r].orientation).get_y(),
//                             i, true});
//     }
//     for (size_t i = 0; i < free_tasks.size(); ++i){
//         const auto t = free_tasks[i];
//         auto& task = env->task_pool[t];
//         assert(task.agent_assigned == -1);
//         xs.push_back({Position(env->task_pool[t].locations[0] + 1, 0).get_x(), i, false});
//         ys.push_back({Position(env->task_pool[t].locations[0] + 1, 0).get_y(), i, false});
//     }
//     // assert(fr_robots == free_robots.size());
//     // assert(free_tasks.size() == fr_tasks);

//     int THREADS_TO_USE = THREADS;
//     if (free_robots.size() * free_robots.size() * free_tasks.size() < 40'000'000) {
//         THREADS_TO_USE = 1;
//     }
//     int DEVIDE_Y = std::sqrt(THREADS_TO_USE);
//     int DEVIDE_X = DEVIDE_Y;

// #ifdef ENABLE_PRINT_LOG
//     Printer() << "free robots: " << free_robots.size() << '\n';
//     Printer() << "free tasks: " << free_tasks.size() << '\n';
// #endif


//     std::sort(xs.begin(), xs.end());
//     std::sort(ys.begin(), ys.end());

//     std::vector<std::pair<size_t, size_t>> robot_distrib(free_robots.size());
//     std::vector<std::pair<size_t, size_t>> task_distrib(free_tasks.size());

//     {
//         const size_t Y_PART_SIZE = (free_robots.size() + DEVIDE_Y - 1) / DEVIDE_Y;
//         size_t curr_size = 0;
//         size_t curr_part = 0;
//         for (size_t i = 0; i < ys.size(); i++) {
//             if (ys[i].is_robot) {
//                 curr_size++;
//             }
//             if (curr_size > Y_PART_SIZE) {
//                 curr_part++;
//                 assert(curr_part < DEVIDE_Y);
//                 curr_size = 0;
//             }

//             if (ys[i].is_robot) {
//                 if (ys[i].index >= robot_distrib.size()){
//                     assert(false);
//                 }
//                 robot_distrib[ys[i].index].first = curr_part;
//             } else {
//                 task_distrib[ys[i].index].first = curr_part;
//             }
//         }
//     }

//     {
//         const size_t X_PART_SIZE = (free_robots.size() + DEVIDE_X - 1) / DEVIDE_X;
//         size_t curr_size = 0;
//         size_t curr_part = 0;
//         for (size_t i = 0; i < xs.size(); i++) {
//             if (xs[i].is_robot) {
//                 curr_size++;
//             }
//             if (curr_size > X_PART_SIZE) {
//                 curr_size = 0;
//                 curr_part++;
//                 assert(curr_part < DEVIDE_X);
//             }

//             if (xs[i].is_robot) {
//                 robot_distrib[xs[i].index].second = curr_part;
//             } else {
//                 task_distrib[xs[i].index].second = curr_part;
//             }
//         }
//     }

//     // {
//     //     const size_t Y_PART_SIZE = (ys.size()+DEVIDE_Y-1)/DEVIDE_Y;
//     //     for (size_t part = 0; part < DEVIDE_Y; ++part){
//     //         #ifdef ENABLE_PRINT_LOG
//     //             int i = part*Y_PART_SIZE;
//     //             if (i < ys.size()){
//     //                 std::cout << "y sep number: " << part << " value: " << ys[i].cordinate << std::endl;
//     //             }
//     //         #endif ENABLE_PRINT_LOG

//     //         for (size_t i = part*Y_PART_SIZE; i < std::min(ys.size(), (part+1)*Y_PART_SIZE); ++i){
//     //             if (ys[i].is_robot){
//     //                 robot_distrib[ys[i].index].first = part;
//     //             } else {
//     //                 task_distrib[ys[i].index].first = part;
//     //             }
//     //         }
//     //     }
//     // }

//     // {
//     //     const size_t X_PART_SIZE = (ys.size()+DEVIDE_X-1)/DEVIDE_X;
//     //     for (size_t part = 0; part < DEVIDE_X; ++part){
//     //         #ifdef ENABLE_PRINT_LOG
//     //             int i = part*X_PART_SIZE;
//     //             if (i < ys.size()){
//     //                 std::cout << "x sep number: " << part << " value: " << xs[i].cordinate << std::endl;
//     //             }
//     //         #endif ENABLE_PRINT_LOG
//     //         for (size_t i = part*X_PART_SIZE; i < std::min(xs.size(), (part+1)*X_PART_SIZE); ++i){
//     //             if (xs[i].is_robot){
//     //                 robot_distrib[xs[i].index].second = part;
//     //             } else {
//     //                 task_distrib[xs[i].index].second = part;
//     //             }
//     //         }
//     //     }
//     // }


//     std::vector devided_robots(DEVIDE_Y, std::vector(DEVIDE_X, std::vector<int>()));
//     std::vector devided_tasks(DEVIDE_Y, std::vector(DEVIDE_X, std::vector<int>()));

//     for (size_t i = 0; i < robot_distrib.size(); ++i) {
//         devided_robots[robot_distrib[i].first][robot_distrib[i].second].push_back(i);
//     }
//     for (size_t i = 0; i < task_distrib.size(); ++i) {
//         assert(task_distrib[i].first != -1);
//         assert(task_distrib[i].second != -1);
//         devided_tasks[task_distrib[i].first][task_distrib[i].second].push_back(i);
//     }

// #ifdef ENABLE_PRINT_LOG

//     for (size_t y = 0; y < DEVIDE_Y; y++) {
//         for (size_t x = 0; x < DEVIDE_X; x++) {
//             Printer() << "Quarant: y: " << y << " x: " << x << '\n';
//             size_t r_s = devided_robots[y][x].size();
//             size_t t_s = devided_tasks[y][x].size();
//             Printer() << "Robots: " << r_s << " (" << ((float) r_s) / (free_robots.size()) * 100 << "%)" << '\n';
//             Printer() << "Tasks:  " << t_s << " (" << ((float) t_s) / (free_tasks.size()) * 100 << "%)" << '\n';
//         }
//     }

// #endif


//     auto RunHungary = [&](size_t y, size_t x) {
//         const auto &tasks = devided_tasks[y][x];
//         const auto &robots = devided_robots[y][x];

//         int rb = min(tasks.size(), robots.size());

//         std::vector<std::vector<int>> dist_matrix(rb + 1, std::vector<int>(tasks.size() + 1, 0));
//         for (int i = 0; i < rb; i++) {
//             for (int g = 0; g < tasks.size(); g++) {
//                 dist_matrix[i + 1][g + 1] = (int) get_dist(free_robots[robots[i]], free_tasks[tasks[g]], env);
//             }
//         }
//         auto ans = Hungarian::DoHungarian(dist_matrix);

//         for (int i = 1; i < ans.size(); i++) {
//             if (ans[i] != -1) {
//                 auto r = free_robots[robots[i - 1]];
//                 auto t = free_tasks[tasks[ans[i] - 1]];
//                 schedule[r] = t;
//             }
//         }
//     };

//     std::vector<std::thread> threads;
//     for (size_t y = 0; y < DEVIDE_Y; y++) {
//         for (size_t x = 0; x < DEVIDE_X; x++) {
//             threads.emplace_back(RunHungary, y, x);
//         }
//     }

//     for (auto &thr: threads) {
//         thr.join();
//     }

//     for (size_t r = 0; r < schedule.size(); r++) {
//         int task_id = schedule[r];
//         if (task_id != -1) {
//             auto dist = get_dist_to_start(r, task_id, env);
//             auto free = jg.IsTaskFree(task_id);
//             // if (dist <= 3){
//             //     std::cout << "LESS" << std::endl;
//             // }
//             // std::cout << " R: " << r << " tid: " << task_id << " " << dist << " " << free << std::endl;
//             if (dist <= 3 && free) {
//                 //std::cout << "CLAIMING: " << r << " " << task_id << "!" << std::endl;
//                 jg.Claim(r, task_id);
//             }
//         }
//     }




// #ifdef ENABLE_PRINT_LOG
//     Printer() << "Scheduler: " << timer << '\n';
// #endif

//     //jg.OutDBG();
// }


std::vector<int> MyScheduler::artem_schedule(int time_limit, std::vector<int> &schedule) {

    Timer timer;

    std::vector<int> done_proposed_schedule = schedule;

    TimePoint end_time = env->plan_start_time + std::chrono::milliseconds(time_limit);


    std::vector<uint32_t> free_robots, free_tasks;

    struct Entity {
        uint32_t cordinate;
        size_t index;
        bool is_robot;
        bool operator<(const Entity &other) const {
            return std::tie(cordinate, index, is_robot) < std::tie(other.cordinate, other.index, other.is_robot);
        }
    };

    std::vector<Entity> xs;
    std::vector<Entity> ys;
    for (uint32_t r = 0; r < env->num_of_agents; r++) {
        uint32_t t = env->curr_task_schedule[r];
        if (t == -1) {
            auto pos = Position(env->curr_states[r].location + 1, env->curr_states[r].orientation);
            xs.push_back({Position(env->curr_states[r].location + 1, env->curr_states[r].orientation).get_x(), free_robots.size(), true});
            ys.push_back({Position(env->curr_states[r].location + 1, env->curr_states[r].orientation).get_y(), free_robots.size(), true});
            free_robots.push_back(r);
        }
    }
    for (auto &[t, task]: env->task_pool) {
        if (task.agent_assigned == -1) {
            xs.push_back({Position(env->task_pool[t].locations[0] + 1, 0).get_x(), free_tasks.size(), false});
            ys.push_back({Position(env->task_pool[t].locations[0] + 1, 0).get_y(), free_tasks.size(), false});
            free_tasks.push_back(t);
        }
    }

    int THREADS_TO_USE = THREADS;
    if (free_robots.size() * free_robots.size() * free_tasks.size() < 40'000'000) {
        THREADS_TO_USE = 1;
    }
    int DEVIDE_Y = sqrt(THREADS_TO_USE);
    int DEVIDE_X = DEVIDE_Y;

#ifdef ENABLE_PRINT_LOG
    Printer() << "free robots: " << free_robots.size() << '\n';
    Printer() << "free tasks: " << free_tasks.size() << '\n';
#endif


    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());

    std::vector<std::pair<size_t, size_t>> robot_distrib(free_robots.size());
    std::vector<std::pair<size_t, size_t>> task_distrib(free_tasks.size());

    {
        const size_t Y_PART_SIZE = (free_robots.size() + DEVIDE_Y - 1) / DEVIDE_Y;
        size_t curr_size = 0;
        size_t curr_part = 0;
        for (size_t i = 0; i < ys.size(); i++) {
            if (ys[i].is_robot) {
                curr_size++;
            }
            if (curr_size > Y_PART_SIZE) {
                curr_size = 0;
                curr_part++;
                assert(curr_part < DEVIDE_Y);
            }

            if (ys[i].is_robot) {
                robot_distrib[ys[i].index].first = curr_part;
            } else {
                task_distrib[ys[i].index].first = curr_part;
            }
        }
    }


    {
        const size_t X_PART_SIZE = (free_robots.size() + DEVIDE_X - 1) / DEVIDE_X;
        size_t curr_size = 0;
        size_t curr_part = 0;
        for (size_t i = 0; i < xs.size(); i++) {
            if (xs[i].is_robot) {
                curr_size++;
            }
            if (curr_size > X_PART_SIZE) {
                curr_size = 0;
                curr_part++;
                assert(curr_part < DEVIDE_X);
            }

            if (xs[i].is_robot) {
                robot_distrib[xs[i].index].second = curr_part;
            } else {
                task_distrib[xs[i].index].second = curr_part;
            }
        }
    }


    // {
    //     const size_t Y_PART_SIZE = (ys.size()+DEVIDE_Y-1)/DEVIDE_Y;
    //     for (size_t part = 0; part < DEVIDE_Y; ++part){
    //         #ifdef ENABLE_PRINT_LOG
    //             int i = part*Y_PART_SIZE;
    //             if (i < ys.size()){
    //                 std::cout << "y sep number: " << part << " value: " << ys[i].cordinate << std::endl;
    //             }
    //         #endif ENABLE_PRINT_LOG

    //         for (size_t i = part*Y_PART_SIZE; i < std::min(ys.size(), (part+1)*Y_PART_SIZE); ++i){
    //             if (ys[i].is_robot){
    //                 robot_distrib[ys[i].index].first = part;
    //             } else {
    //                 task_distrib[ys[i].index].first = part;
    //             }
    //         }
    //     }
    // }

    // {
    //     const size_t X_PART_SIZE = (ys.size()+DEVIDE_X-1)/DEVIDE_X;
    //     for (size_t part = 0; part < DEVIDE_X; ++part){
    //         #ifdef ENABLE_PRINT_LOG
    //             int i = part*X_PART_SIZE;
    //             if (i < ys.size()){
    //                 std::cout << "x sep number: " << part << " value: " << xs[i].cordinate << std::endl;
    //             }
    //         #endif ENABLE_PRINT_LOG
    //         for (size_t i = part*X_PART_SIZE; i < std::min(xs.size(), (part+1)*X_PART_SIZE); ++i){
    //             if (xs[i].is_robot){
    //                 robot_distrib[xs[i].index].second = part;
    //             } else {
    //                 task_distrib[xs[i].index].second = part;
    //             }
    //         }
    //     }
    // }


    std::vector devided_robots(DEVIDE_Y, std::vector(DEVIDE_X, std::vector<int>()));
    std::vector devided_tasks(DEVIDE_Y, std::vector(DEVIDE_X, std::vector<int>()));

    for (size_t i = 0; i < robot_distrib.size(); ++i) {
        devided_robots[robot_distrib[i].first][robot_distrib[i].second].push_back(i);
    }

    for (size_t i = 0; i < task_distrib.size(); ++i) {
        assert(task_distrib[i].first != -1);
        assert(task_distrib[i].second != -1);
        devided_tasks[task_distrib[i].first][task_distrib[i].second].push_back(i);
    }

#ifdef ENABLE_PRINT_LOG

    for (size_t y = 0; y < DEVIDE_Y; y++) {
        for (size_t x = 0; x < DEVIDE_X; x++) {
            Printer() << "Quarant: y: " << y << " x: " << x << '\n';
            size_t r_s = devided_robots[y][x].size();
            size_t t_s = devided_tasks[y][x].size();
            Printer() << "Robots: " << r_s << " (" << ((float) r_s) / (free_robots.size()) * 100 << "%)" << '\n';
            Printer() << "Tasks:  " << t_s << " (" << ((float) t_s) / (free_tasks.size()) * 100 << "%)" << '\n';
        }
    }

#endif

    // std::vector<int> free_tasks_length(free_tasks.size());
    // for (int i = 0; i < free_tasks.size(); ++i){
    //     free_tasks_length[i] = calc_full_distance(env->task_pool[free_tasks[i]]);
    // }

    auto RunHungary = [&](size_t y, size_t x) {
        const auto &tasks = devided_tasks[y][x];
        const auto &robots = devided_robots[y][x];

        int rb = min(tasks.size(), robots.size());

        std::vector<std::vector<int>> dist_matrix(rb + 1, std::vector<int>(tasks.size() + 1, 0));
        for (int i = 0; i < rb; i++) {
            for (int g = 0; g < tasks.size(); g++) {
                // + free_tasks_length[tasks[g]];
                dist_matrix[i + 1][g + 1] = (int) get_dist(free_robots[robots[i]], free_tasks[tasks[g]], env);
            }
        }
        auto ans = Hungarian::DoHungarian(dist_matrix);

        for (int i = 1; i < ans.size(); i++) {
            if (ans[i] != -1) {
                auto r = free_robots[robots[i - 1]];
                auto t = free_tasks[tasks[ans[i] - 1]];
                schedule[r] = t;
            }
        }
    };


    std::vector<std::thread> threads;
    for (size_t y = 0; y < DEVIDE_Y; y++) {
        for (size_t x = 0; x < DEVIDE_X; x++) {
            threads.push_back(std::thread(RunHungary, y, x));
        }
    }

    for (auto &thr: threads) {
        thr.join();
    }

    for (size_t r = 0; r < schedule.size(); r++) {
        int task_id = schedule[r];
        if (task_id != -1) {
            if (get_dist_to_start(r, task_id, env) <= 3) {
                done_proposed_schedule[r] = task_id;
            }
        }
    }

#ifdef ENABLE_PRINT_LOG
    Printer() << "Scheduler: " << timer << '\n';
#endif

    return done_proposed_schedule;
}




void MyScheduler::plan(int time_limit, std::vector<int> &proposed_schedule) {

    /*{
        std::set<uint32_t> used_tasks;
        for (uint32_t r = 0; r < env->num_of_agents; r++) {
            if (env->curr_task_schedule[r] != -1) {
                ASSERT(!used_tasks.count(env->curr_task_schedule[r]), "already contains 0");
                used_tasks.insert(env->curr_task_schedule[r]);
            }
        }
    }*/

    Timer timer;
    //solver_schedule(time_limit, proposed_schedule);
    // auto res = greedy_schedule(time_limit, proposed_schedule);
    //auto old_schedule = proposed_schedule;
    //auto res = solver_schedule(time_limit, proposed_schedule); // 5324
    // auto res = greedy_schedule(time_limit, proposed_schedule); // 5237
    // auto res = greedy_schedule_double(time_limit, proposed_schedule);
    artem_schedule(time_limit, proposed_schedule);

    /*std::set<uint32_t> used_tasks;
    for (uint32_t r = 0; r < env->num_of_agents; r++) {
        if (proposed_schedule[r] != -1) {
            ASSERT(!used_tasks.count(proposed_schedule[r]), "already contains 1");
            used_tasks.insert(proposed_schedule[r]);
        }
    }

    auto it = env->task_pool.begin();
    int cnt = 0;
    for (uint32_t r = 0; r < env->num_of_agents; r++) {
        if (proposed_schedule[r] == -1) {
            // need to find free task
            while (it != env->task_pool.end() && (used_tasks.count(it->first) || it->second.idx_next_loc != 0)) {
                it++;
            }
            ASSERT(it != env->task_pool.end(), "failed to get free task");
            ASSERT(!used_tasks.count(it->first), "already contains 2");
            proposed_schedule[r] = it->first;
            used_tasks.insert(it->first);
            //it->second.agent_assigned = r;
            cnt++;
        }
    }

    Printer() << "Added tasks: " << cnt << '\n';

    {
        std::set<uint32_t> used_tasks;
        for (uint32_t r = 0; r < env->num_of_agents; r++) {
            if (proposed_schedule[r] != -1) {
                ASSERT(!used_tasks.count(proposed_schedule[r]), "already contains 3");
                used_tasks.insert(proposed_schedule[r]);
            }
        }
    }*/

    /*{
        std::vector<int> greedy_ans = old_schedule;
        greedy_schedule(time_limit, greedy_ans);
        double greedy_score = 0;
        for (uint32_t r = 0; r < greedy_ans.size(); r++) {
            if (old_schedule[r] != -1) {
                continue;
            }
            if (greedy_ans[r] != -1) {
                greedy_score += get_dist(r, greedy_ans[r], env);
            } else {
                greedy_score += 1e6;
            }
        }
        Printer() << "solver score: " << solver.get_score() << '\n';
        Printer() << "greedy score: " << greedy_score << '\n';
        if (std::abs(greedy_score - solver.get_score()) > 1e-9) {
            Printer() << "diff solver scores!\n";
        }
        if (solver.get_score() - 1e-9 > greedy_score) {
            Printer() << "bad\n";
        }
        if (solver.get_score() - 1e-9 < greedy_score) {
            Printer() << "good\n";
        }
    }*/
#ifdef ENABLE_PRINT_LOG
    Printer() << "Scheduler: " << timer << '\n';
#endif
}
