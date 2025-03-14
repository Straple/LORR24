#pragma once

#include <Objects/Basic/randomizer.hpp>
#include <Planner/epibt.hpp>

// Enhanced Priority Inheritance with BackTracking + Large Neighborhood Search
class EPIBT_LNS : public EPIBT {

    double old_score = 0;

    double temp = 1;

    uint32_t visited_counter = 1;

    std::vector<uint32_t> visited;

    uint32_t pibt_step = 0;

    Randomizer rnd;

    bool consider();

    enum class RetType {
        FAILED,
        ACCEPTED,// success + accepted
        REJECTED,// success + not accepted
    };

    RetType try_build(uint32_t r, uint32_t &counter, uint32_t depth);

    bool try_build(uint32_t r);

    RetType build(uint32_t r, uint32_t depth, uint32_t &counter);

    bool build(uint32_t r);

public:
    EPIBT_LNS(const std::vector<Robot> &robots, TimePoint end_time);

    void solve(uint64_t seed);

    [[nodiscard]] uint32_t get_step() const;
};
