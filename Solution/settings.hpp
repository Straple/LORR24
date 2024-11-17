#pragma once

#include <cstdint>
#include <set>

/*
steps | my PIBT | my PIBT + dynamic dists |   MAPFPlanner
500   |   823   |          900            |       910
1000  |  1613   |         1762            |      1783
10000 | 14132   |        16582            |     16833
*/

// python3 PlanViz/script/run.py --map example_problems/random.domain/maps/random-32-32-20.map --plan test.json --end 1000

// -i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 1000 -t 500 -p 10000

#define ENABLE_ASSERT

//#define ENABLE_THEIR_HEURISTIC

// if disabled then use manhattan heuristic (very bad), without build dist matrix
#define ENABLE_DIST_MATRIX

#define ENABLE_DYNAMICS_DIST_MATRIX

static constexpr uint32_t UPDATE_DYNAMICS_DIST_MATRIX_TIME = 400;

//#define ENABLE_PLANNER_SOLVER

#define ENABLE_PIBT

//#define ENABLE_PIBT_SOLVER

//#define ENABLE_PLANNER_MACHINE

static constexpr uint32_t THREADS = 4;

static constexpr uint32_t PLANNER_DEPTH = 3;

static constexpr uint32_t SPLIT_ROBOTS_BOUND = 30;

struct EPlanner;   // мой алгоритм
struct MAPFPlanner;// их алгоритм
using PLANNER = EPlanner;

struct MyScheduler;  // мой алгоритм
struct TaskScheduler;// их алгоритм
using TASKSHEDULLER = TaskScheduler;
