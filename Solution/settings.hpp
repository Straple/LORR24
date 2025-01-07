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

#define ENABLE_ASSERT

#define ENABLE_HEURISTIC_MATRIX

#define ENABLE_PIBT

#define ENABLE_DHM

// при завершении программы вызывает tools::build_meta_info в driver.cpp
//#define BUILD_META_INFO

#define ENABLE_PRINT_LOG

// -i ./example_problems/game.domain/brc202d_500.json -o test.json -s 1000 -t 10000 -p 100000000

static constexpr uint32_t THREADS = 4;

static constexpr uint32_t PLANNER_DEPTH = 3;

// if -1, then use timer
// else use steps, without timer
static constexpr uint32_t PIBTS_STEPS = 1000;

static constexpr uint32_t DHM_TIMELIMIT = 10000;

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

//PIBTS_STEPS = 500
//call(0): 2545, 13.7082s
//call(1): 4367, 29.2802s
//call(2): 5235, 35.5224s
//call(3): 5180, 46.8324s
//call(4): 4255, 70.5182s
//call(5): 3456, 111.651s
//total: 25038
//
//PIBTS_STEPS = 1000
//call(0): 2537, 26.0555s
//call(1): 4398, 35.2493s
//call(2): 5337, 42.8796s
//call(3): 5574, 57.9827s
//call(4): 4427, 104.332s
//call(5): 3509, 170.459s
//total: 25782