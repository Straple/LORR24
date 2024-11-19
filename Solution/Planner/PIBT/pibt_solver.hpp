#pragma once
#ifdef MY_UNUSED
#include "pibt.hpp"

class PIBTSolver {
    std::vector<uint32_t> order;

public:
    PIBTSolver();

    std::vector<Action> solve(std::chrono::steady_clock::time_point end_time);
};
#endif