#include <Scheduler/scheduler.hpp>

#include <Scheduler/scheduler_solver.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>

#include <thread>
#include <unordered_set>

void MyScheduler::initialize(int preprocess_time_limit) {
}

void MyScheduler::plan(int time_limit, std::vector<int> &proposed_schedule) {
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint end_time = env->plan_start_time + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    static std::vector<uint32_t> free_robots, free_tasks;
    // update free robots and tasks
    {
        for (uint32_t r: env->new_freeagents) {
            free_robots.push_back(r);
        }

        for (uint32_t t: env->new_tasks) {
            free_tasks.push_back(t);
        }

        /*{
            std::set<uint32_t> s(free_robots.begin(), free_robots.end());
            ASSERT(s.size() == free_robots.size(), "invalid s");
        }
        {
            std::set<uint32_t> s(free_tasks.begin(), free_tasks.end());
            ASSERT(s.size() == free_tasks.size(), "invalid s");
        }*/
    }

    if (free_robots.empty() || free_tasks.empty()) {
        return;
    }

    auto get_dist = [&](uint32_t r, uint32_t t) {
        std::array<uint64_t, 4> data{};
        constexpr static uint64_t MAX_SCORE = 1e15;
        {
            uint32_t source = get_graph().get_node(
                    Position(env->curr_states[r].location, env->curr_states[r].orientation));
            uint32_t loc = env->task_pool[t].locations[0];
            for (uint32_t dir = 0; dir < 4; dir++) {
                Position p(loc, dir);
                uint32_t target = get_graph().get_node(p);
                data[dir] =//get_h(get_graph().get_pos(robots[r].node).get_pos(), loc);
                        get_hm().get(source, target);
            }
        }
        for (uint32_t i = 0; i + 1 < env->task_pool[t].locations.size(); i++) {
            std::array<uint64_t, 4> new_data{MAX_SCORE, MAX_SCORE, MAX_SCORE, MAX_SCORE};
            for (uint32_t src_dir = 0; src_dir < 4; src_dir++) {
                for (uint32_t dst_dir = 0; dst_dir < 4; dst_dir++) {
                    Position source_pos(env->task_pool[t].locations[i], src_dir);
                    uint32_t source_node = get_graph().get_node(source_pos);

                    Position target_pos(env->task_pool[t].locations[i + 1], dst_dir);
                    uint32_t target_node = get_graph().get_node(target_pos);

                    uint64_t dist =//get_h(source_pos.get_pos(), target_pos.get_pos());
                            get_hm().get(source_node, target_node);
                    new_data[dst_dir] = std::min(new_data[dst_dir], data[src_dir] + dist);
                }
            }
            data = new_data;
        }

        return *std::min_element(data.begin(), data.end());
    };

    // for each task calculate best robot
    while (!free_robots.empty()) {
        if (std::chrono::steady_clock::now() >= end_time) {
            break;
        }

        uint32_t r = free_robots.back();

        std::vector<std::pair<uint64_t, uint32_t>> answers(THREADS, {-1, -1});

        auto do_work = [&](uint32_t thr) {
            uint32_t best_id = -1;
            uint64_t best_dist = -1;
            for (uint32_t j = thr; j < free_tasks.size(); j += THREADS) {
                if (std::chrono::steady_clock::now() >= end_time) {
                    break;
                }
                uint32_t t = free_tasks[j];
                uint64_t dist = get_dist(r, t);
                if (dist < best_dist) {
                    best_dist = dist;
                    best_id = j;
                }
            }
            answers[thr] = {best_dist, best_id};
        };

        std::vector<std::thread> threads(THREADS);
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            threads[thr] = std::thread(do_work, thr);
        }
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            threads[thr].join();
        }

        auto [best_dist, best_id] = *std::min_element(answers.begin(), answers.end());
        if (best_id == -1) {
            break;
        }

        proposed_schedule[r] = free_tasks[best_id];

        free_robots.pop_back();

        std::swap(free_tasks[best_id], free_tasks.back());
        free_tasks.pop_back();
    }

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    //std::unordered_set<int> free_agents(env->new_freeagents.begin(), env->new_freeagents.end());
    //std::unordered_set<int> free_tasks(env->new_tasks.begin(), env->new_tasks.end());

    /*for (int r = 0; r > env->num_of_agents; r++) {
        int t = env->curr_task_schedule[r];
        if (t == -1) {
            ASSERT(free_agents.count(r), "no contains");
        } else {
            if (env->task_pool[t].idx_next_loc == 0) {
                free_agents.insert(r);
                free_tasks.insert(t);
            }
        }
    }*/

    /*for (auto [id, task]: env->task_pool) {
        ASSERT(id == task.task_id, "invalid id");
        if (task.agent_assigned == -1) {
            free_tasks.insert(id);
        }
    }

    int min_task_i, min_task_makespan, dist, c_loc, count;

    std::cout << "kek: " << free_agents.size() << ' ' << free_tasks.size() << ' ' << env->task_pool.size() << std::endl;

    if (free_agents.empty() || free_tasks.empty()) {
        return;
    }*/

    //5000000000000
    //4970000013258

    // CALL SOLVER HERE

    /*std::vector<SchedulerSolver> ss(THREADS);

    auto do_work = [&](uint32_t thr, uint64_t seed) {
        ss[thr].solve(*env, seed, endtime);
    };

    static Randomizer rnd;

    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(do_work, thr, rnd.get());
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }

    uint32_t best_thr = 0;
    uint64_t best_score = -1;
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        //std::cout << ss[thr].get_score() << ' ';
        if (best_score > ss[thr].get_score()) {
            best_score = ss[thr].get_score();
            best_thr = thr;
        }
    }
    //std::cout << std::endl;

    ss[best_thr].set_schedule(proposed_schedule);*/


    // iterate over the free agents to decide which task to assign to each of them
    /*auto it = free_agents.begin();
    while (it != free_agents.end()) {
        //keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime) {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;
        min_task_makespan = INT_MAX;
        count = 0;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id: free_tasks) {
            //check for timeout every 10 task evaluations
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime) {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // iterate over the locations (errands) of the task to compute the makespan to finish the task
            // makespan: the time for the agent to complete all the errands of the task t_id in order
            for (int loc: env->task_pool[t_id].locations) {
                uint32_t source = get_graph().get_node(Position(c_loc, 0));
                uint32_t dest = get_graph().get_node(Position(loc, 0));
                dist += get_hm().get(source, dest);//DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            // update the new minimum makespan
            if (dist < min_task_makespan) {
                min_task_i = t_id;
                min_task_makespan = dist;
            }
            count++;
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1) {
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
        }
        // nothing to assign
        else {
            proposed_schedule[i] = -1;
            it++;
        }
    }*/
}
