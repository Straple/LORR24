#pragma once

#include <cstdint>
#include <fstream>
#include <iostream>
#include <set>

// поиск по [KEK], чтобы найти всякие моменты

//#define ENABLE_DEFAULT_SCHEDULER

#define ENABLE_ASSERT

#define ENABLE_GG

#define BUILD_META_INFO

#define PRINT(args...) \
    do { args } while (0)

#define ENABLE_LNS_ANNEALING

#define ENABLE_SCHEDULER_CHANGE_TASK

static constexpr uint32_t THREADS = 32;

static constexpr uint32_t SCHEDULER_REBUILD_DP_TIME = 300;

static constexpr uint32_t SCHEDULER_LAZY_SOLVE_TIME = 150;

static constexpr uint32_t SCHEDULER_LNS_SOLVE_TIME = 0;

uint32_t &get_unique_id();

#define ENABLE_FILEPRINT

struct Printer {
    [[nodiscard]] std::ofstream &get() const;
};

template<typename T>
Printer operator<<(Printer printer, const T &value) {
#ifdef ENABLE_FILEPRINT
    //printer.get() << value;
    //printer.get().flush();
    std::cout << value;// << std::flush;
#else
    std::cout << value;
#endif
    return printer;
}
