#pragma once

#include <cstdint>
#include <fstream>
#include <iostream>
#include <set>

//#define ENABLE_DEFAULT_PLANNER

//#define ENABLE_DEFAULT_SCHEDULER

//#define ENABLE_TRIVIAL_SCHEDULER

//#define ENABLE_ASSERT

#define ENABLE_HEURISTIC_MATRIX

#define ENABLE_PIBTS

//#define ENABLE_PMPS

#define ENABLE_DHM

// при завершении программы вызывает tools.cpp::build_meta_info в driver.cpp
//#define BUILD_META_INFO

//#define ENABLE_PRINT_LOG

#define ENABLE_PIBTS_ANNEALING

#define ENABLE_PIBTS_TRICK

//#define ENABLE_GG_SOLVER

// если включено, то шедулер не будет трогать робота и задачу,
// если они менялись несколько шагов назад
//#define ENABLE_SCHEDULER_FREEZE

static constexpr uint32_t MAX_CONST = 10'000'000;

static constexpr uint32_t THREADS = 32;

// if -1, then use timer
// else use steps, without timer
static constexpr uint32_t PIBTS_STEPS = 500;

static constexpr uint32_t DHM_REBUILD_TIMELIMIT = 200;

static constexpr uint32_t DHM_REBUILD_COUNT = MAX_CONST;

static constexpr uint32_t SCHEDULER_REBUILD_DP_TIME = 200;

static constexpr uint32_t SCHEDULER_TRIV_SOLVE_TIME = 200;

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
