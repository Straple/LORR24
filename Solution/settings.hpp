#pragma once

#include <cstdint>

// python3 PlanViz/script/run2.py --map example_problems/random.domain/maps/random-32-32-20.map --plan test.json --end 1000

// -i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 1000 -t 500 -p 10000

//#define ENABLE_THEIR_HEURISTIC

// if disabled then use manhattan heuristic (very bad), without build dist matrix
#define ENABLE_DIST_MATRIX

//#define ENABLE_ASSERT

//#define ENABLE_PLANNER_SOLVER

#define ENABLE_PIBT

//#define ENABLE_PLANNER_MACHINE

static constexpr uint32_t THREADS = 31;

static constexpr uint32_t PLANNER_DEPTH = 3;

static constexpr uint32_t SPLIT_ROBOTS_BOUND = 30;

struct EPlanner;   // мой алгоритм
struct MAPFPlanner;// их алгоритм
using PLANNER = EPlanner;

struct MyScheduler;  // мой алгоритм
struct TaskScheduler;// их алгоритм
using TASKSHEDULLER = TaskScheduler;
