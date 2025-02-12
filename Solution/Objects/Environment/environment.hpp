#pragma once

#include <Objects/Environment/dynamic_heuristic_matrix.hpp>
#include <Objects/Environment/dhmr.hpp>
#include <Objects/Environment/graph.hpp>
#include <Objects/Environment/graph_guidance.hpp>
#include <Objects/Environment/guidance_map.hpp>
#include <Objects/Environment/heuristic_matrix.hpp>
#include <Objects/Environment/map.hpp>
#include <Objects/Environment/operations.hpp>
#include <Objects/Environment/operations_map.hpp>
#include <Objects/Environment/robot_handler.hpp>
#include <Objects/Environment/info.hpp>
#include <Objects/Environment/workload_map.hpp>

#include <SharedEnv.h>
#include <ActionModel.h>

void init_environment(SharedEnvironment &env);

void update_environment(SharedEnvironment &env);

std::vector<Action> &get_myplan();
