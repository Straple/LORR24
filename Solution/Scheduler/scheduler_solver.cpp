#include <Scheduler/scheduler_solver.hpp>

#include <Objects/Environment/graph.hpp>

#include <unordered_set>

bool SchedulerSolver::compare(uint64_t old, uint64_t cur, Randomizer &rnd) {
    return old >= cur;
}

uint64_t SchedulerSolver::get_score(uint32_t r, uint32_t t) const {
    std::array<uint64_t, 4> data{};
    {
        uint32_t loc = tasks[t].path[0];
        for (uint32_t dir = 0; dir < 4; dir++) {
            Position p(loc, dir);
            uint32_t target = get_graph().get_node(p);
            data[dir] =//get_h(get_graph().get_pos(robots[r].node).get_pos(), loc);
                    get_hm().get(robots[r].node, target);
        }
    }
    for (uint32_t i = 0; i + 1 < tasks[t].path.size(); i++) {
        std::array<uint64_t, 4> new_data{MAX_SCORE, MAX_SCORE, MAX_SCORE, MAX_SCORE};
        for (uint32_t src_dir = 0; src_dir < 4; src_dir++) {
            for (uint32_t dst_dir = 0; dst_dir < 4; dst_dir++) {
                Position source_pos(tasks[t].path[i], src_dir);
                uint32_t source_node = get_graph().get_node(source_pos);

                Position target_pos(tasks[t].path[i + 1], dst_dir);
                uint32_t target_node = get_graph().get_node(target_pos);

                uint64_t dist =//get_h(source_pos.get_pos(), target_pos.get_pos());
                        get_hm().get(source_node, target_node);
                new_data[dst_dir] = std::min(new_data[dst_dir], data[src_dir] + dist);
            }
        }
        data = new_data;
    }

    return *std::min_element(data.begin(), data.end());
}

void SchedulerSolver::remove_task(uint32_t r) {
    if (robots[r].task_id == -1) {
        return;
    }

    total_score -= robots[r].score;

    tasks[robots[r].task_id].robot_id = -1;
    robots[r].task_id = -1;

    robots[r].score = MAX_SCORE;
    total_score += MAX_SCORE;
}

void SchedulerSolver::add_task(uint32_t r, uint32_t t) {
    if (robots[r].task_id != -1) {
        remove_task(r);
    }

    ASSERT(tasks[t].robot_id == -1, "already used task");
    tasks[t].robot_id = r;

    total_score -= MAX_SCORE;
    robots[r].task_id = t;

    robots[r].score = get_score(r, t);
    total_score += robots[r].score;
}

bool SchedulerSolver::try_change(Randomizer &rnd) {
    uint64_t old_score = total_score;

    uint32_t r = rnd.get(0, robots.size() - 1);
    uint32_t t = rnd.get(0, tasks.size() - 1);

    uint32_t old_task_id = robots[r].task_id;
    if (old_task_id == t) {
        return false;
    }

    if (old_task_id != -1) {
        remove_task(r);
    }

    uint32_t old_robot_id = tasks[t].robot_id;
    if (old_robot_id != -1) {
        remove_task(old_robot_id);

        if (old_task_id != -1) {
            add_task(old_robot_id, old_task_id);
        }
    }

    add_task(r, t);

    return consider(old_score, rnd, [&]() {
        remove_task(r);

        if (old_robot_id != -1) {
            add_task(old_robot_id, t);
        }

        if (old_task_id != -1) {
            add_task(r, old_task_id);
        }

        ASSERT(robots[r].task_id == old_task_id, "invalid task_id");
        if (old_robot_id != -1) {
            ASSERT(robots[old_robot_id].task_id == t, "invalid task_id");
        }
    });
}

uint64_t SchedulerSolver::get_h(uint32_t source, uint32_t dest) const {
    return DefaultPlanner::get_h(env_ptr, source, dest);
}

void SchedulerSolver::solve(SharedEnvironment &env, const TimePoint end_time, std::vector<int> &proposed_schedule) {
    env_ptr = &env;

    std::unordered_set<uint32_t> free_agents(env.new_freeagents.begin(), env.new_freeagents.end());
    for (uint32_t r = 0; r < env.num_of_agents; r++) {
        if (env.curr_task_schedule[r] == -1) {
            free_agents.insert(r);
        }
    }

    for (uint32_t r: free_agents) {
        robots.push_back({r, get_graph().get_node(Position(env.curr_states[r].location, env.curr_states[r].orientation))});
        total_score += robots.back().score;
    }

    std::unordered_set<uint32_t> free_tasks(env.new_tasks.begin(), env.new_tasks.end());
    for (auto [id, task]: env.task_pool) {
        ASSERT(id == task.task_id, "invalid id");
        if (task.agent_assigned == -1) {
            free_tasks.insert(id);
        }
    }

    //std::cout << "kek: " << free_agents.size() << ' ' << free_tasks.size() << ' ' << env.task_pool.size() << std::endl;

    if (free_agents.empty() || free_tasks.empty()) {
        return;
    }

    for (uint32_t t: free_tasks) {
        std::vector<uint32_t> path;
        for (int loc: env.task_pool[t].locations) {
            path.push_back(loc);
        }
        tasks.push_back({t, path});
    }

    Randomizer rnd;

    //std::cout << "score: " << total_score;
    for (uint32_t step = 0;; step++) {

        if (step % 10 == 0) {
            if (std::chrono::steady_clock::now() >= end_time) {
                break;
            }
        }
        try_change(rnd);
    }
    //std::cout << "->" << total_score << std::endl;

    for (auto &robot: robots) {
        if (robot.task_id != -1) {
            proposed_schedule[robot.id] = tasks[robot.task_id].id;
        }
    }
}
