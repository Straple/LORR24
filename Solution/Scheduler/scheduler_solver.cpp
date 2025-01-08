#include <Scheduler/scheduler_solver.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>

bool SchedulerSolver::compare(double cur_score, double old_score, Randomizer &rnd) const {
    return cur_score <= old_score || rnd.get_d() < std::exp((old_score - cur_score) / temp);
}

double SchedulerSolver::get_dist(uint32_t r, uint32_t t) const {
    if (t == -1) {
        return 1e6;
    }
    uint32_t source = get_graph().get_node(Position(env->curr_states[r].location + 1, env->curr_states[r].orientation));
    //ASSERT(std::find(free_tasks.begin(), free_tasks.end(), t) != free_tasks.end(), "task is not contains");
    ASSERT(env->task_pool[t].idx_next_loc == 0, "invalid idx next loc");
    uint32_t loc = env->task_pool[t].locations[0] + 1;
    return get_dhm().get(source, loc);
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
    uint32_t new_t = rnd.get(free_tasks);

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
    if (desires.size() != env->num_of_agents) {
        desires.resize(env->num_of_agents, -1);
    }
    free_robots.clear();
    free_tasks.clear();
    for (uint32_t r = 0; r < env->num_of_agents; r++) {
        uint32_t t = env->curr_task_schedule[r];
        if (t == -1) {
            free_robots.push_back(r);
        }
    }

    for (auto &[t, task]: env->task_pool) {
        if (task.agent_assigned == -1) {
            ASSERT(env->task_pool[t].idx_next_loc == 0, "invalid idx next loc");
            free_tasks.push_back(t);
        }
    }

    cur_score = 0;
    for (uint32_t r: free_robots) {
        //if (desires[r] != -1) {
        //    task_to_robot[desires[r]] = -1;
        //}
        //desires[r] = -1;

        if (desires[r] != -1 && !env->task_pool.count(desires[r])) {
            desires[r] = -1;
        }

        cur_score += get_dist(r, desires[r]);
    }
    validate();

#ifdef ENABLE_PRINT_LOG
    Printer() << "free robots: " << free_robots.size() << '\n';
    Printer() << "free tasks: " << free_tasks.size() << '\n';
#endif
}

void SchedulerSolver::solve(TimePoint end_time) {
    if (free_robots.empty() || free_tasks.empty()) {
        return;
    }
    static Randomizer rnd;
    temp = 1;
    Timer timer;
    for (uint32_t step = 0; step < 1'000'000; step++) {
        try_peek_task(rnd);
        //try_smart(rnd);
        temp *= 0.9999;
    }
    Printer() << "SchedulerSolver: " << timer << ", " << get_score() << '\n';
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
