#pragma once

#include <cstdint>

// python3 PlanViz/script/run2.py --map example_problems/random.domain/maps/random-32-32-20.map --plan test.json --end 1000

// if disabled then use manhattan heuristic (very bad), without build dist matrix
#define ENABLE_DIST_MATRIX

//#define ENABLE_ASSERT

static constexpr uint32_t THREADS = 31;

static constexpr uint32_t PLANNER_DEPTH = 4;

static constexpr uint32_t SPLIT_ROBOTS_BOUND = 30;

struct EPlanner;// мой алгоритм
// struct MAPFPlanner; // их алгоритм
using PLANNER = EPlanner;

struct TaskScheduler;// их алгоритм
using TASKSHEDULLER = TaskScheduler;
