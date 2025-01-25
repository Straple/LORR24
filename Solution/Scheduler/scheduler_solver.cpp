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
    Timer timer;
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
    Printer() << "SchedulerSolver::rebuild_dp: " << counter << "/" << order.size() << ", " << timer << '\n';
#endif
}

bool SchedulerSolver::compare(double cur_score, double old_score, Randomizer &rnd) const {
    return cur_score <= old_score || rnd.get_d() < std::exp((old_score - cur_score) / temp);
}

uint32_t SchedulerSolver::get_dist(uint32_t r, uint32_t t) const {
    if (t == -1) {
        return 1e6;
    }
    uint32_t dist = 0;
    uint32_t source = get_graph().get_node(Position(env->curr_states[r].location + 1, env->curr_states[r].orientation));
    for (int i = 0; i < env->task_pool[t].locations.size(); i++) {
        int loc = env->task_pool[t].locations[i];
        dist += get_hm().get(source, loc + 1);// or hm?
        source = get_graph().get_node(Position(loc + 1, env->curr_states[r].orientation));
    }
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
    timestep_changed_task.resize(desires.size(), -1);

    free_robots.clear();
    free_tasks.clear();

    /*for (uint32_t r = 0; r < env->num_of_agents; r++) {
        // задача уже все
        if (desires[r] != -1 && env->task_pool.count(desires[r]) == 0) {
            desires[r] = -1;
            free_robots.push_back(r);
        }
    }*/

    uint32_t SCHEDULER_TIMESTEP_DIFF = 0;

#ifdef ENABLE_SCHEDULER_FREEZE
    if (get_map_type() == MapType::WAREHOUSE) {
        SCHEDULER_TIMESTEP_DIFF = 10;
    } else if (get_map_type() == MapType::SORTATION) {
        SCHEDULER_TIMESTEP_DIFF = 10;
    } else if (get_map_type() == MapType::GAME) {
        SCHEDULER_TIMESTEP_DIFF = 15;
    }
#endif

    old_desires = desires;

    for (uint32_t r = 0; r < env->num_of_agents; r++) {
        int t = env->curr_task_schedule[r];

        // есть задача и она в процессе выполнения
        // не можем ее убрать
        if (env->task_pool.count(t) && env->task_pool.at(t).idx_next_loc != 0) {
            continue;
        }
        if (
                // нет задачи
                !env->task_pool.count(t) ||
                // есть задача, но мы давно не обновляли ее
                (timestep_changed_task[r] == -1 || env->curr_timestep - timestep_changed_task[r] >= SCHEDULER_TIMESTEP_DIFF)

                //#ifndef ENABLE_TRIVIAL_SCHEDULER
                //|| env->task_pool.at(t).idx_next_loc == 0
                //#endif
        ) {
            free_robots.push_back(r);
        }
    }

#ifndef ENABLE_TRIVIAL_SCHEDULER
    for (auto &[t, task]: env->task_pool) {
        /*if (task.agent_assigned != -1 &&
            //timestep_changed_task[task.agent_assigned] != -1 &&
            //env->curr_timestep - timestep_changed_task[task.agent_assigned] < 5
            ) {
            continue;
        }*/
        int r = task.agent_assigned;
        if (
                // нет агента
                r == -1 ||
                // иначе он есть, НО
                (task.idx_next_loc == 0 &&// мы можем поменять задачу
                 // и хорошо по timestep
                 (timestep_changed_task[r] == -1 || env->curr_timestep - timestep_changed_task[r] >= SCHEDULER_TIMESTEP_DIFF)//
                 )

        ) {
            task.agent_assigned = -1;// IMPORTANT! remove task agent assigned
            free_tasks.push_back(t);
        }
    }
    //ASSERT(env->task_pool[t].idx_next_loc == 0, "invalid idx next loc");

    cur_score = 0;
    for (uint32_t r: free_robots) {
        desires[r] = -1;
        cur_score += get_dist(r, desires[r]);
    }
    validate();
#endif

#ifdef ENABLE_PRINT_LOG
    Printer() << "free robots: " << free_robots.size() << '\n';
    Printer() << "free tasks: " << free_tasks.size() << '\n';
#endif
}

void SchedulerSolver::triv_solve(TimePoint end_time) {
    Timer timer;
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
            Heap.push({dp[r][0].first, r, 0});
        }
    }

    while (!Heap.empty() && get_now() < end_time) {
        auto [dist, r, index] = Heap.top();
        Heap.pop();

        uint32_t task_id = dp[r][index].second;
        ASSERT(dist == dp[r][index].first, "invalid dist");

        // not used in this timestep
        if (used_task.count(task_id)
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
        ASSERT(!used_task.count(task_id), "already used");

        set(r, task_id);
        used_task.insert(task_id);
    }

    for (uint32_t r: free_robots) {
        if (desires[r] != old_desires[r] && old_desires[r] != -1) {
            timestep_changed_task[r] = env->curr_timestep;
        }
    }
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
    Timer timer;
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
    for (uint32_t i = 0; i < desires.size(); i++) {
        result[i] = static_cast<int>(desires[i]);
    }
    return result;
}

double SchedulerSolver::get_score() const {
    return cur_score;
}
