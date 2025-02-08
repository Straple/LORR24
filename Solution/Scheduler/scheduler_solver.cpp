#include <Scheduler/scheduler_solver.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>

#include <atomic>
#include <thread>

void SchedulerSolver::rebuild_dp(uint32_t r) {
    dp[r].clear();
    for (uint32_t t: free_tasks) {
        dp[r].emplace_back(get_dist(r, t), t);
    }
    std::sort(dp[r].begin(), dp[r].end());
    timestep_updated[r] = env->curr_timestep;
}

void SchedulerSolver::rebuild_dp(TimePoint end_time) {
    ETimer timer;
    std::vector<uint32_t> order = free_robots;
    std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
        return timestep_updated[lhs] < timestep_updated[rhs];
    });

    std::atomic<uint32_t> counter{};
    auto do_work = [&](uint32_t thr) {
        for (uint32_t i = thr; i < order.size() && get_now() < end_time; i += THREADS) {
            rebuild_dp(order[i]);
            ++counter;
        }
    };

    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(do_work, thr);
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }
#ifdef ENABLE_PRINT_LOG
    Printer() << "SchedulerSolver::rebuild_dp: " << counter << "/" << order.size() << " ("
              << counter * 100.0 / order.size() << "%), " << timer << '\n';
#endif
}

bool SchedulerSolver::compare(double cur_score, double old_score, Randomizer &rnd) const {
    return cur_score <= old_score || rnd.get_d() < std::exp((old_score - cur_score) / temp);
}

uint32_t SchedulerSolver::get_dist(uint32_t r, uint32_t t) const {
    if (t == -1) {
        return 1e6;
    }

    uint32_t source = get_robots_handler().get_robot(r).node;
    uint32_t dist_to_target = get_hm().get(source, task_target[t]);
    uint32_t dist = dist_to_target * dist_to_target + dist_dp[t]; // 6387
    //uint32_t dist = dist_to_target + dist_dp[t] * dist_dp[t]; // 6001
    //uint32_t dist = dist_to_target + dist_dp[t]; // 6201

    //uint32_t dist = dist_to_target * 5 + dist_dp[t];  // 6262
    //uint32_t dist = dist_to_target * 10 + dist_dp[t]; // 6399
    //uint32_t dist = dist_to_target * 13 + dist_dp[t]; // 6195
    //uint32_t dist = dist_to_target * 15 + dist_dp[t]; // 6416
    //uint32_t dist = dist_to_target * 16 + dist_dp[t]; // 6466
    //uint32_t dist = dist_to_target * 17 + dist_dp[t]; // 6372
    //uint32_t dist = dist_to_target * 20 + dist_dp[t]; // 6283

    return dist;
}

void SchedulerSolver::set(uint32_t r, uint32_t t) {
    if (desires[r] != -1) {
        task_to_robot[desires[r]] = -1;
    }
    cur_score -= get_dist(r, desires[r]);
    desires[r] = t;
    if (t != -1) {
        task_to_robot[t] = r;
    }
    cur_score += get_dist(r, desires[r]);
}

bool SchedulerSolver::try_peek_task(Randomizer &rnd) {
    double old_score = cur_score;

    uint32_t r = rnd.get(free_robots);
    uint32_t new_t = -1;

    if (dp[r].empty() || rnd.get_d() < 0.2) {
        new_t = rnd.get(free_tasks);
    } else {
        new_t = dp[r][rnd.get(0, std::min(dp[r].size(), static_cast<size_t>(dp[r].size() * 0.2 + 1)))].second;
        if (!env->task_pool.count(new_t) || env->task_pool[new_t].agent_assigned != -1) {
            new_t = rnd.get(free_tasks);
        }
    }

    uint32_t old_t = desires[r];
    uint32_t other_r = task_to_robot[new_t];
    if (other_r != -1) {
        set(other_r, -1);
        set(r, new_t);
        set(other_r, old_t);
    } else {
        set(r, new_t);
    }
    validate();

    return consider(old_score, rnd, [&]() {
        if (other_r != -1) {
            set(r, -1);
            set(other_r, new_t);
            set(r, old_t);
        } else {
            set(r, old_t);
        }
        validate();
    });
}

bool SchedulerSolver::try_smart(Randomizer &rnd) {
    double old_score = cur_score;

    uint32_t r = rnd.get(free_robots);
    //uint32_t new_t = rnd.get(free_tasks);
    uint32_t new_t = free_tasks[0];
    for (uint32_t t: free_tasks) {
        if (get_dist(r, new_t) > get_dist(r, t)) {
            new_t = t;
        }
    }

    uint32_t old_t = desires[r];
    uint32_t other_r = task_to_robot[new_t];
    if (other_r != -1) {
        set(other_r, -1);
        set(r, new_t);
        set(other_r, old_t);
    } else {
        set(r, new_t);
    }
    validate();

    return consider(old_score, rnd, [&]() {
        if (other_r != -1) {
            set(r, -1);
            set(other_r, new_t);
            set(r, old_t);
        } else {
            set(r, old_t);
        }
        validate();
    });
}

void SchedulerSolver::validate() {
    /*std::set<uint32_t> S;
    for (uint32_t r = 0; r < desires.size(); r++) {
        if (desires[r] != -1) {
            ASSERT(!S.count(desires[r]), "already contains");
            S.insert(desires[r]);
            ASSERT(task_to_robot[desires[r]] == r, "invalid task to robot");
        }
    }*/
}

SchedulerSolver::SchedulerSolver(SharedEnvironment *env)
        : env(env), desires(env->num_of_agents, -1), task_to_robot(500'000, -1) {
}

void SchedulerSolver::update() {
    desires.resize(env->num_of_agents, -1);
    timestep_updated.resize(desires.size());
    dp.resize(desires.size());
    phantom_agent_dist.assign(desires.size(), 0);

    free_robots.clear();
    free_tasks.clear();

    // build free_robots
    for (uint32_t r = 0; r < env->num_of_agents; r++) {
        int t = env->curr_task_schedule[r];

        // есть задача и она в процессе выполнения
        // не можем ее убрать
        if (env->task_pool.count(t) && env->task_pool.at(t).idx_next_loc != 0) {
#ifndef ENABLE_PHANTOM_SCHEDULE
            continue;
#endif
            const auto &task = env->task_pool.at(t);

            uint32_t dist = get_hm().get(get_robots_handler().get_robot(r).node,
                                         get_robots_handler().get_robot(r).target);

            // если у него последняя точка задачи
            if (task.idx_next_loc + 1 == task.locations.size() &&
                // и если он не в таргете стоит
                env->curr_states[r].location != task.locations.back() &&
                dist < PHANTOM_AGENT_AVAILABLE_DIST
                // TODO: и ему совсем скоро выполнить задачу
                    ) {

                free_robots.push_back(r);
                phantom_agent_dist[r] = dist;
                ASSERT(phantom_agent_dist[r] != 0, "invalid phantom agent dist");
            }

            desires[r] = t;
            continue;
        }
        if (
            // нет задачи
                !env->task_pool.count(t)
                //#ifndef ENABLE_TRIVIAL_SCHEDULER
                || env->task_pool.at(t).idx_next_loc == 0
            //#endif
                ) {
            free_robots.push_back(r);
        }
    }

    // build free_tasks
    for (auto &[t, task]: env->task_pool) {
        int r = task.agent_assigned;
        if (
                r == -1 || // нет агента
                task.idx_next_loc == 0 // мы можем поменять задачу
                ) {
            task.agent_assigned = -1;// IMPORTANT! remove task agent assigned
            free_tasks.push_back(t);
        }
    }
    //ASSERT(env->task_pool[t].idx_next_loc == 0, "invalid idx next loc");

    // build dist_dp, task_target
    for (uint32_t t: free_tasks) {
        if (dist_dp.size() <= t) {
            dist_dp.resize(t + 1, -1);
            task_target.resize(t + 1);
        }
        auto &task = env->task_pool[t];
        task_target[t] = task.locations[0] + 1;
        if (dist_dp[t] == -1) {
            uint32_t d = 0;
            for (int i = 0; i + 1 < task.locations.size(); i++) {
                int source = task.locations[i] + 1;
                int target = task.locations[i + 1] + 1;
                d += get_hm().get(get_graph().get_node(Position(source, 0)), target);
            }
            dist_dp[t] = d;
        }
    }

    for (uint32_t t: free_tasks) {
        task_to_robot[t] = -1;
    }

    cur_score = 0;
    for (uint32_t r: free_robots) {
        desires[r] = -1;
        cur_score += get_dist(r, desires[r]);
        cur_score += phantom_agent_dist[r];
    }
    validate();

#ifdef ENABLE_PRINT_LOG
    Printer() << "free robots: " << free_robots.size() << '\n';
    Printer() << "free tasks: " << free_tasks.size() << '\n';
#endif
}

void SchedulerSolver::triv_solve(TimePoint end_time) {
    ETimer timer;
    for (uint32_t r: free_robots) {
        if (desires[r] != -1) {
            set(r, -1);
        }
    }
#ifdef ENABLE_TRIVIAL_SCHEDULER
    for (uint32_t r: free_robots) {
        uint32_t task_id = r;
        while (!env->task_pool.count(task_id)) {
            task_id += env->num_of_agents;
            if (task_id > 1'000'000'000) {
                Printer() << "pizda: " << r << ' ' << task_id << ' ' << env->task_pool.size() << ' ';
                //Printer().get().flush();
                //_exit(0);
                //throw "kek";
                break;
            }
            //ASSERT(task_id < 1e8, "invalid task_id");
        }
        if (env->task_pool.count(task_id)) {
            set(r, task_id);
        }
    }
    Printer() << '\n';
#else
    std::unordered_set<uint32_t> used_task;

    // (dist, r, index)
    std::priority_queue<std::tuple<uint32_t, uint32_t, uint32_t>, std::vector<std::tuple<uint32_t, uint32_t, uint32_t>>, std::greater<>> Heap;
    for (uint32_t r: free_robots) {
        if (!dp[r].empty()) {
            Heap.push({dp[r][0].first + phantom_agent_dist[r], r, 0});
        }
    }

    int32_t allowed_assigned = 0;
    {
        int32_t cnt_assigned = 0;
        for (uint32_t r = 0; r < desires.size(); r++) {
            int t = env->curr_task_schedule[r];
            if (t != -1 && env->task_pool.count(t) && env->task_pool.at(t).idx_next_loc != 0) {
                // этот робот уже с задачей, которую нельзя менять
                cnt_assigned++;
            }
        }

        int32_t max_assigned = desires.size();

        if (get_test_type() == TestType::RANDOM_5) {
            max_assigned = 300;
        } else if (get_test_type() == TestType::RANDOM_4) {
            max_assigned = 200;
        } else if (get_test_type() == TestType::GAME) {
            max_assigned = 2500;
        }

        allowed_assigned = std::max(0, max_assigned - cnt_assigned);

        //Printer() << "cnt_assigned: " << cnt_assigned << '\n';
        //Printer() << "max_assigned: " << max_assigned << '\n';
        //Printer() << "allowed_assigned: " << allowed_assigned << '\n';
    }

    while (!Heap.empty() && get_now() < end_time && allowed_assigned > 0) {
        auto [dist, r, index] = Heap.top();
        Heap.pop();

        uint32_t task_id = dp[r][index].second;
        ASSERT(dist == dp[r][index].first + phantom_agent_dist[r], "invalid dist");

        // not used in this timestep
        if (used_task.count(task_id)
            // this task is available
            || !env->task_pool.count(task_id)
            // robot already used this task
            || env->task_pool[task_id].agent_assigned != -1) {

            if (index + 1 < dp[r].size()) {
                Heap.push({dp[r][index + 1].first + phantom_agent_dist[r], r, index + 1});
            }

            continue;
        }

        ASSERT(env->task_pool.count(task_id), "no contains");
        ASSERT(env->task_pool[task_id].agent_assigned == -1, "already assigned");
        ASSERT(!used_task.count(task_id), "already used");

        set(r, task_id);
        used_task.insert(task_id);

        if (phantom_agent_dist[r] == 0) {
            allowed_assigned--;
        }
    }

    validate();

    /*auto it = free_tasks.begin();
    for (uint32_t r = 0; r < desires.size(); r++) {
        if (desires[r] == -1) {
            while (it != free_tasks.end() && task_to_robot[*it] != -1) {
                it++;
            }
            ASSERT(it != free_tasks.end(), "unable to set task");
            set(r, *it);
        }
    }*/

#endif

#ifdef ENABLE_PRINT_LOG
    Printer() << "SchedulerSolver::triv_solve: " << timer << '\n';
#endif
}

void SchedulerSolver::solve(TimePoint end_time) {
    if (free_robots.empty() || free_tasks.empty()) {
        return;
    }
    static Randomizer rnd;
    temp = 0.5;
    ETimer timer;
    double old_score = get_score();
    uint32_t step = 0;
    // TODO: multithreading
    for (; get_now() < end_time; step++) {
        try_peek_task(rnd);
        //try_smart(rnd);
        temp *= 0.999;
    }
#ifdef ENABLE_PRINT_LOG
    Printer() << "SchedulerSolver::solve: " << old_score << "->" << get_score() << ", " << step << ", " << timer
              << '\n';
#endif
}

std::vector<int> SchedulerSolver::get_schedule() const {
    std::vector<int> result(desires.size());
    for (uint32_t r = 0; r < desires.size(); r++) {
        if (phantom_agent_dist[r]) {
            result[r] = env->curr_task_schedule[r];
        } else {
            result[r] = static_cast<int>(desires[r]);
        }
    }
    return result;
}

double SchedulerSolver::get_score() const {
    return cur_score;
}
