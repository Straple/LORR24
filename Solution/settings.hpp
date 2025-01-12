#pragma once

#include <cstdint>
#include <fstream>
#include <iostream>
#include <set>

/*
steps | my PIBT | my PIBT + dynamic dists |   MAPFPlanner
500   |   823   |          900            |       910
1000  |  1613   |         1762            |      1783 -> 2186 -> 2209
10000 | 14132   |        16582            |     17042->18055
*/

// ssh -i ../abc egor@51.250.101.48

// scp -i ../abc -r * egor@51.250.101.48:/home/egor

// ./compile.sh

// ./build/lifelong -i ./example_problems/warehouse.domain/warehouse_large_5000.json -o test.json -s 1000 -t 300 -p 1800000

// ./build/lifelong -i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 1000 -t 300 -p 1800000

// ./build/lifelong -i ./example_problems/my.domain/random_32_32_20_100.json -o test.json -s 1000 -t 300 -p 1800000

// python3 PlanViz/script/run2.py --map example_problems/random.domain/maps/random-32-32-20.map --plan test.json --end 1000

// -i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 1000 -t 500 -p 1800000

//#define ENABLE_ASSERT

#define ENABLE_HEURISTIC_MATRIX

#define ENABLE_PIBT

#define ENABLE_DHM

// при завершении программы вызывает tools.cpp::build_meta_info в driver.cpp
//#define BUILD_META_INFO

//#define ENABLE_PRINT_LOG

#define ENABLE_PIBTS_ANNEALING

//#define ENABLE_GG_SOLVER

// -i ./example_problems/game.domain/brc202d_500.json -o test.json -s 1000 -t 10000 -p 100000000

static constexpr uint32_t THREADS = 32;

static constexpr uint32_t PLANNER_DEPTH = 3;

// if -1, then use timer
// else use steps, without timer
static constexpr uint32_t PIBTS_STEPS = -1;

static constexpr uint32_t DHM_TIMELIMIT = 400;

static constexpr uint32_t DHM_REBUILD_COUNT = -1;

struct EPlanner;   // мой алгоритм
struct MAPFPlanner;// их алгоритм
using PLANNER = EPlanner;

struct MyScheduler;  // мой алгоритм
struct TaskScheduler;// их алгоритм
using TASKSHEDULLER = MyScheduler;

#define ENABLE_SCHEDULER_TRICK

static constexpr uint32_t INVALID_DIST = 0;

uint32_t &get_unique_id();

#define ENABLE_FILEPRINT

struct Printer {
    [[nodiscard]] std::ofstream &get() const;
};

template<typename T>
Printer operator<<(Printer printer, const T &value) {
#ifdef ENABLE_FILEPRINT
    printer.get() << value;
    std::cout << value;
#else
    std::cout << value;
#endif
    return printer;
}


/*
call(0): 2352, 19.4394s
call(1): 2325, 20.138s
call(2): 4137, 50.8091s
call(3): 4081, 57.113s
call(4): 5238, 51.5664s
call(5): 5177, 57.8458s
call(6): 5761, 67.9453s
call(7): 5543, 67.6235s
call(8): 4973, 140.395s
call(9): 5207, 111.707s
call(10): 4078, 304.588s // PIBTS_STEPS = 0: 165.8s 3802, если без rnd.get_d() < 0.8, то 91.5006s 3075
call(11): 4004, 306.231s // TODO: тут оочень медленно с PIBTS_STEPS=1000, попробовать поменять
total: 52876
TODO: попробовать поменять try_build так, чтобы он стремился на начальных агентах улучшать скор,\
 а если это не так, то с некоторой вероятностью он выйдет

PIBTS_STEPS = 1000
258.399s 3667
175.168s 3834
*/

