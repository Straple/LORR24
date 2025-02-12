#include <Scheduler/scheduler_solver.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>

#include <Planner/Default/default_planner_solver.hpp>
#include <Planner/eplanner.hpp>
#include <planner.h>

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
    PRINT(
            Printer() << "SchedulerSolver::rebuild_dp: " << counter << "/" << order.size() << " ("
                      << counter * 100.0 / order.size() << "%), " << timer << '\n';);
}

bool SchedulerSolver::compare(double cur_score, double old_score, Randomizer &rnd) const {
    return cur_score <= old_score || rnd.get_d() < std::exp((old_score - cur_score) / temp);
}

uint64_t SchedulerSolver::get_dist(uint32_t r, uint32_t t) const {
    if (t == -1) {
        return 1e6;
    }

    uint32_t source = get_robots_handler().get_robot(r).node;
    uint64_t dist_to_target = get_hm().get(source, task_target[t]);
    // uint64_t dist = dist_to_target * dist_to_target + dist_dp[t]; // 41680
    uint64_t dist = dist_to_target + dist_dp[t]; // 40350
    return dist;
}

void SchedulerSolver::remove(uint32_t r) {
    ASSERT(0 <= r && r < desires.size(), "invalid r");
    uint32_t t = desires[r];
    if (t == -1) {
        return;
    }

    cur_score -= get_dist(r, t);
    task_to_robot[t] = -1;
    desires[r] = -1;
}

void SchedulerSolver::add(uint32_t r, uint32_t t) {
    ASSERT(0 <= r && r < desires.size(), "invalid r");
    ASSERT(0 <= t && t < task_to_robot.size(), "invalid t");
    ASSERT(desires[r] == -1, "already have task");
    ASSERT(task_to_robot[t] == -1, "already have robot");

    cur_score += get_dist(r, t);
    task_to_robot[t] = r;
    desires[r] = t;
}

bool SchedulerSolver::try_peek_task(Randomizer &rnd) {
    double old_score = cur_score;

    uint32_t r = rnd.get(free_robots);
    uint32_t t = rnd.get(free_tasks);

    // уже используется
    if (desires[r] == t) {
        return false;
    }

    //auto old_desires = desires;
    //auto old_task_to_robot = task_to_robot;

    uint32_t old_t = desires[r];
    uint32_t other_r = task_to_robot[t];
    ASSERT(r != other_r, "invalid other_r");
    if (other_r != -1) {
        remove(r);
        remove(other_r);

        add(r, t);
        if (old_t != -1) {
            add(other_r, old_t);
        }
    } else {
        remove(r);
        add(r, t);
    }
    validate();

    return consider(old_score, rnd, [&]() {
        if (other_r != -1) {
            remove(r);
            remove(other_r);

            if (old_t != -1) {
                add(r, old_t);
            }
            add(other_r, t);
        } else {
            remove(r);
            if (old_t != -1) {
                add(r, old_t);
            }
        }
        validate();

        //ASSERT(desires == old_desires, "invalid desires");
        //ASSERT(task_to_robot == old_task_to_robot, "invalid task_to_robot");
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
    : env(env), desires(env->num_of_agents, -1), task_to_robot(1'000'000, -1) {
}

void SchedulerSolver::update() {
    desires.resize(env->num_of_agents, -1);
    timestep_updated.resize(desires.size());
    dp.resize(desires.size());
    phantom_agent_dist.assign(desires.size(), 0);

    free_robots.clear();
    free_tasks.clear();

    // build free_tasks
    for (auto &[t, task]: env->task_pool) {
        int r = task.agent_assigned;
        if (
                r == -1 ||            // нет агента
                task.idx_next_loc == 0// мы можем поменять задачу
        ) {
            task.agent_assigned = -1;// IMPORTANT! remove task agent assigned
            free_tasks.push_back(t);
        }
    }

    // build free_robots
    for (uint32_t r = 0; r < env->num_of_agents; r++) {
        int t = env->curr_task_schedule[r];

        // есть задача и она в процессе выполнения
        // не можем ее убрать
        if (env->task_pool.count(t) && env->task_pool.at(t).idx_next_loc != 0) {
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
#ifdef ENABLE_PHANTOM_SCHEDULE
    for (uint32_t r = 0; r < env->num_of_agents; r++) {
        int t = env->curr_task_schedule[r];

        // есть задача и она в процессе выполнения
        // не можем ее убрать
        if (env->task_pool.count(t) && env->task_pool.at(t).idx_next_loc != 0) {
            const auto &task = env->task_pool.at(t);

            uint32_t dist = get_hm().get(get_robots_handler().get_robot(r).node,
                                         get_robots_handler().get_robot(r).target);

            // если у него последняя точка задачи
            if (task.idx_next_loc + 1 == task.locations.size() &&
                // и если он не в таргете стоит
                env->curr_states[r].location != task.locations.back() &&
                dist < PHANTOM_AGENT_AVAILABLE_DIST &&
                free_robots.size() + 1 <= free_tasks.size()) {

                free_robots.push_back(r);
                phantom_agent_dist[r] = dist;
                ASSERT(phantom_agent_dist[r] != 0, "invalid phantom agent dist");
            }
        }
    }
#endif

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
                //get_wmap().get(get_graph().get_node(Position(source, 0)), target)
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

    PRINT(
            Printer() << "free robots: " << free_robots.size() << '\n';
            Printer() << "free tasks: " << free_tasks.size() << '\n';);
}

void SchedulerSolver::triv_solve(TimePoint end_time) {
    ETimer timer;
    for (uint32_t r: free_robots) {
        remove(r);
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
            max_assigned = 400;
        } else if (get_test_type() == TestType::RANDOM_4) {
            max_assigned = 300;
        } else if (get_test_type() == TestType::GAME) {
            max_assigned = 2500;// 4752
            // without: 3227
        }

        allowed_assigned = std::max(0, max_assigned - cnt_assigned);

        PRINT(
                Printer() << "cnt_assigned: " << cnt_assigned << '\n';
                Printer() << "max_assigned: " << max_assigned << '\n';
                Printer() << "allowed_assigned: " << allowed_assigned << '\n';);
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

        add(r, task_id);
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

    PRINT(Printer() << "SchedulerSolver::triv_solve: " << timer << '\n';);
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
    for (; get_now() < end_time; step++) {
        try_peek_task(rnd);
        temp *= 0.999;
    }
    PRINT(
            Printer() << "SchedulerSolver::solve: " << old_score << "->" << get_score() << ", " << step << ", " << timer
                      << '\n';);
}

namespace DefaultPlanner {
    extern std::vector<int> decision;
    extern std::vector<int> prev_decision;
    extern std::vector<double> p;
    extern std::vector<State> prev_states;
    extern std::vector<State> next_states;
    extern std::vector<int> ids;
    extern std::vector<double> p_copy;
    extern std::vector<bool> occupied;
    extern std::vector<DefaultPlanner::DCR> decided;
    extern std::vector<bool> checked;
    extern std::vector<bool> require_guide_path;
    extern std::vector<int> dummy_goals;
    extern DefaultPlanner::TrajLNS trajLNS;
}// namespace DefaultPlanner

std::vector<int> SchedulerSolver::get_schedule(TimePoint end_time) const {
    std::vector<int> result(desires.size());
    for (uint32_t r = 0; r < desires.size(); r++) {
        if (phantom_agent_dist[r]) {
            result[r] = env->curr_task_schedule[r];
        } else {
            result[r] = static_cast<int>(desires[r]);
        }
    }
#ifdef ENABLE_SCHEDULER_TRICK
    if (get_test_type() == TestType::SORTATION ||
        get_test_type() == TestType::WAREHOUSE) {
        env->curr_task_schedule = result;
        update_environment(*env);
        EPlanner eplanner(env);
        auto [plan, desires_plan] = eplanner.plan(std::min(end_time, get_now() + Milliseconds(SCHEDULER_TRICK_TIME)));

        // тут берется просто путь минимальный
        // ооочень плохо 26647 -> 18279
        /*std::vector<uint32_t> desires_plan(desires.size());
        const auto & robots = get_robots_handler().get_robots();
        for(uint32_t r = 0; r < desires.size(); r++){
            // (priority, desired)
            std::vector<std::pair<int64_t, uint32_t>> steps;
            for (uint32_t desired = 1; desired < get_operations().size(); desired++) {
                if (!get_omap().get_poses_path(robots[r].node, desired)[0]) {
                    continue;
                }
                int64_t priority = get_hm().get(get_omap().get_nodes_path(robots[r].node, desired).back(), robots[r].target);
                steps.emplace_back(priority, desired);
            }
            std::sort(steps.begin(), steps.end());
            desires_plan[r] = steps[0].second;
        }*/

        //get_myplan().resize(desires.size());
        for (uint32_t r = 0; r < desires.size(); r++) {
            //get_myplan()[r] = get_operations()[desires_plan[r]][0];

            uint32_t source = get_robots_handler().get_robot(r).node;
            const auto &poses = get_omap().get_poses_path(source, desires_plan[r]);
            const auto &nodes = get_omap().get_nodes_path(source, desires_plan[r]);
            Operation op = get_operations()[desires_plan[r]];
            uint32_t to = poses.back();
            //for (uint32_t i = 0; i < poses.size(); i++) {
            //    if (op[i] == Action::FW) {
            //        to = poses[i];
            //        break;
            //    }
            //}

            to = get_graph().get_pos_from_zip(to);
            ASSERT(get_map().is_free(to), "is not free");

            int t = result[r];
            if (t == -1) {
                continue;
            }
            auto &task = env->task_pool.at(t);

            task.locations.insert(task.locations.begin() + task.idx_next_loc, to - 1);
        }
    }
#endif
    // build order and power
    /*{
        env->curr_task_schedule = result;
        get_robots_handler().update(*env);

        const auto &robots = get_robots_handler().get_robots();
        ETimer timer;
        std::vector<uint32_t> order(robots.size());
        iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
            double lhs_x = robots[lhs].target ? robots[lhs].priority : 1e9;
            double rhs_x = robots[rhs].target ? robots[rhs].priority : 1e9;
            return lhs_x < rhs_x;
        });

        std::vector<int32_t> weight(robots.size());

        for (uint32_t i = 0; i < robots.size(); i++) {
            weight[order[i]] = i;
        }
        int32_t max_weight = robots.size() + 1;

        const double workload = robots.size() * 1.0 / get_map().get_count_free();
        for (uint32_t r = 0; r < robots.size(); r++) {
            double power = (max_weight - weight[r]) * 1.0 / max_weight;
            if (!robots[r].target) {
                power = 0;
            }
            ASSERT(0 <= r && r < DefaultPlanner::p.size(), "invalid r");
            DefaultPlanner::p[r] = power * 100;
            //robot_power[r] = power;//get_robots_handler().get_robot(r).priority;//power;
        }

        Printer() << "init order and power: " << timer << '\n';
    }*/
    // 4228

    /*{
        DefaultPlanner::decision.clear();
        DefaultPlanner::prev_decision.clear();
        DefaultPlanner::p.clear();
        DefaultPlanner::prev_states.clear();
        DefaultPlanner::next_states.clear();
        DefaultPlanner::ids.clear();
        DefaultPlanner::p_copy.clear();
        DefaultPlanner::occupied.clear();
        DefaultPlanner::decided.clear();
        DefaultPlanner::checked.clear();
        DefaultPlanner::require_guide_path.clear();
        DefaultPlanner::dummy_goals.clear();
        DefaultPlanner::trajLNS.~TrajLNS();

        DefaultPlanner::initialize(0, env);
    }*/
    return result;
}

double SchedulerSolver::get_score() const {
    return cur_score;
}
