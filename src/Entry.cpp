#include "Entry.h"
#include "Tasks.h"
#include "heuristics.h"
#include "utils.h"

#include <Objects/Environment/environment.hpp>
#include <Objects/Environment/graph.hpp>
#include <Objects/Environment/heuristic_matrix.hpp>
#include <Objects/Environment/map.hpp>

#include <Planner/PIBT/pibt2.hpp>
#include <Planner/PIBT/pibts.hpp>

#include <Tools/tools.hpp>

#include <settings.hpp>

#include <Objects/Basic/randomizer.hpp>

// The initialize function will be called by competition system at the preprocessing stage.
// Implement the initialize functions of the planner and scheduler to load or compute auxiliary data.
// Note that, this function runs untill preprocess_time_limit (in milliseconds) is reached.
// This is an offline step, after it completes then evaluation begins.
void Entry::initialize(int preprocess_time_limit) {
    get_map() = Map(*env);
    get_graph() = Graph(get_map());
    get_hm() = HeuristicMatrix(get_graph());

    scheduler->initialize(preprocess_time_limit);
    planner->initialize(preprocess_time_limit);

    PIBTS::actions = PIBT2::actions = BuilderActions().get();

    /*Randomizer rnd(5340000);
    std::ofstream output("agents.txt");
    std::set<uint32_t> S;
    for(int i = 0; i < 600; i++){
        uint32_t pos = 0;
        while(true){
            pos = rnd.get(1, get_map().get_size() - 1);
            if(get_map().is_free(pos) && !S.count(pos)){
                break;
            }
        }
        S.insert(pos);
    }
    for(uint32_t pos : S){
        pos--;
        output << pos << '\n';
    }
    output.flush();
    std::exit(0);*/
}

//The compute function will be called by competition system on each timestep.
//It computes:
//  1. a schedule that specifies which agent complete which task.
//  2. a next action that specifies how each agent should move in the next timestep.
//NB: the parameter time_limit is specified in milliseconds.
void Entry::compute(int time_limit, std::vector<Action> &plan, std::vector<int> &proposed_schedule) {
    //static Timer total_timer;
#ifdef ENABLE_SCHEDULER_TRICK
    std::vector<int> done_proposed_schedule =
#endif
            //call the task scheduler to assign tasks to agents
            scheduler->plan(time_limit * 0.7, proposed_schedule);

    //then update the first unfinished errand/location of tasks for planner reference
    update_goal_locations(proposed_schedule);

    //call the planner to compute the actions
    planner->plan(time_limit, plan);

#ifdef ENABLE_SCHEDULER_TRICK
    proposed_schedule = std::move(done_proposed_schedule);
    update_goal_locations(proposed_schedule);
#endif
}

// Set the next goal locations for each agent based on the proposed schedule
void Entry::update_goal_locations(std::vector<int> &proposed_schedule) {
    // record the proposed schedule so that we can tell the planner
    env->curr_task_schedule = proposed_schedule;

    // The first unfinished errand/location of each task is the next goal for the assigned agent.
    for (size_t i = 0; i < proposed_schedule.size(); i++) {
        env->goal_locations[i].clear();
        int t_id = proposed_schedule[i];
        if (t_id == -1)
            continue;

        int i_loc = env->task_pool[t_id].idx_next_loc;
        env->goal_locations[i].push_back({env->task_pool[t_id].locations.at(i_loc), env->task_pool[t_id].t_revealed});
    }
}
