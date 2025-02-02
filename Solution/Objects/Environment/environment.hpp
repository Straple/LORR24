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

void init_environment(SharedEnvironment &env);

void update_environment(SharedEnvironment &env);
