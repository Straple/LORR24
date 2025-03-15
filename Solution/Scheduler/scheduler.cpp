#include <Scheduler/journey_graph.hpp>
#include <Scheduler/scheduler.hpp>

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
        dist += get_hm().get(source, loc + 1);
        source = get_graph().get_node(Position(loc + 1, env->curr_states[r].orientation));
    }
    return dist;
}

void MyScheduler::solver_schedule(TimePoint end_time, std::vector<int> &proposed_schedule) {
    solver.update();
    solver.rebuild_dp(std::min(end_time, get_now() + Milliseconds(SCHEDULER_REBUILD_DP_TIME)));
    solver.lazy_solve(std::min(end_time, get_now() + Milliseconds(SCHEDULER_LAZY_SOLVE_TIME)));
    solver.lns_solve(std::min(end_time, get_now() + Milliseconds(SCHEDULER_LNS_SOLVE_TIME)));
    proposed_schedule = solver.get_schedule();
}

int calc_full_distance(Task &task) {
    auto &hm = get_hm();
    int dist_sum = 0;
    for (size_t i = 0; i + 1 < task.locations.size(); ++i) {
        uint32_t from = task.locations[i] + 1;
        uint32_t to = task.locations[i + 1] + 1;
        FAILED_ASSERT("from must be graph node (INCORRECT), to must be map node (CORRECT)");
        dist_sum += hm.get(from, to);
    }
    return dist_sum;
}

std::vector<int> MyScheduler::artem_schedule(int time_limit, std::vector<int> &schedule) {

    ETimer timer;

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
            xs.push_back({Position(env->curr_states[r].location + 1, env->curr_states[r].orientation).get_x(),
                          free_robots.size(), true});
            ys.push_back({Position(env->curr_states[r].location + 1, env->curr_states[r].orientation).get_y(),
                          free_robots.size(), true});
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

    PRINT(
            Printer() << "free robots: " << free_robots.size() << '\n';
            Printer() << "free tasks: " << free_tasks.size() << '\n';);


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
    //             int i = part*Y_PART_SIZE;
    //             if (i < ys.size()){
    //                 std::cout << "y sep number: " << part << " value: " << ys[i].cordinate << std::endl;
    //             }

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
    //             int i = part*X_PART_SIZE;
    //             if (i < ys.size()){
    //                 std::cout << "x sep number: " << part << " value: " << xs[i].cordinate << std::endl;
    //             }
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


    PRINT(
            for (size_t y = 0; y < DEVIDE_Y; y++) {
                for (size_t x = 0; x < DEVIDE_X; x++) {
                    Printer() << "Quarant: y: " << y << " x: " << x << '\n';
                    size_t r_s = devided_robots[y][x].size();
                    size_t t_s = devided_tasks[y][x].size();
                    Printer() << "Robots: " << r_s << " (" << ((float) r_s) / (free_robots.size()) * 100 << "%)"
                              << '\n';
                    Printer() << "Tasks:  " << t_s << " (" << ((float) t_s) / (free_tasks.size()) * 100 << "%)" << '\n';
                }
            });

    std::vector<int> free_tasks_length(free_tasks.size());
    for (int i = 0; i < free_tasks.size(); ++i) {
        free_tasks_length[i] = calc_full_distance(env->task_pool[free_tasks[i]]);
    }

    auto RunHungary = [&](size_t y, size_t x) {
        const auto &tasks = devided_tasks[y][x];
        const auto &robots = devided_robots[y][x];

        int rb = min(tasks.size(), robots.size());

        std::vector<std::vector<int>> dist_matrix(rb + 1, std::vector<int>(tasks.size() + 1, 0));
        for (int i = 0; i < rb; i++) {
            for (int g = 0; g < tasks.size(); g++) {
                dist_matrix[i + 1][g + 1] = (int) get_dist(free_robots[robots[i]], free_tasks[tasks[g]], env) +
                                            free_tasks_length[tasks[g]];
                ;
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

    return done_proposed_schedule;
}

void MyScheduler::plan(TimePoint end_time, std::vector<int> &proposed_schedule) {
    solver_schedule(end_time, proposed_schedule);
    //artem_schedule(end_time, proposed_schedule);
}
