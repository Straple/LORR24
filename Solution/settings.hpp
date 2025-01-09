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

// -i ./example_problems/game.domain/brc202d_500.json -o test.json -s 1000 -t 10000 -p 100000000

static constexpr uint32_t THREADS = 32;

static constexpr uint32_t PLANNER_DEPTH = 3;

// if -1, then use timer
// else use steps, without timer
static constexpr uint32_t PIBTS_STEPS = -1;

static constexpr uint32_t DHM_TIMELIMIT = 500;

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
DHM
call(0): 2407, 13.9454s
call(1): 4184, 21.6022s
call(2): 5168, 26.6417s
call(3): 5325, 49.4565s
call(4): 4557, 67.1475s
call(5): 3678, 104.547s
total: 25319

new DHM
call(0): 2415, 50.557s
call(1): 4194, 42.81s
call(2): 5280, 34.3922s
call(3): 5702, 57.9102s
call(4): 4835, 101.92s
call(5): 4113, 180.863s
total: 26539

without DHM
call(0): 2362, 9.07632s
call(1): 3880, 14.8847s
call(2): 4068, 35.6515s
call(3): 3366, 79.524s
call(4): 2971, 130.35s
call(5): 2690, 215.291s
total: 19337
*/
