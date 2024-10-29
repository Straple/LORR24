#include "eplanner.hpp"

EPlanner::EPlanner(SharedEnvironment *env) : env(env) {}
EPlanner::EPlanner() {
    env = new SharedEnvironment();
}

void EPlanner::initialize(int preprocess_time_limit){
    // TODO
}

// return next states for all agents
void EPlanner::plan(int time_limit, std::vector<Action> &plan){
    // TODO:
}
