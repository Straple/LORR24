#pragma once

#include "Objects/Basic/randomizer.hpp"
#include "guidance_graph.hpp"

constexpr uint32_t COLS = 32;
constexpr uint32_t ROWS = 32;

class GuidanceGraphSolver {
    GuidanceGraph gg;
    std::string params;

    int64_t cur_score;

    int64_t best_score = 0;

    bool compare(int64_t old, int64_t cur, Randomizer &rnd);

    template<typename rollback_t>
    bool consider(int64_t old, Randomizer &rnd, rollback_t &&rollback) {
        cur_score = get_score();
        std::cout << cur_score << std::endl;
        if (compare(old, cur_score, rnd)) {
            return true;
        } else {
            rollback();
            //ASSERT(old == get_score(), "invalid rollback");
            cur_score = old;
            return false;
        }
    }

    bool try_change_ver_line(Randomizer &rnd);

    bool try_change_gor_line(Randomizer &rnd);

public:
    GuidanceGraphSolver(const GuidanceGraph &gg, const std::string &params);

    void solve();

    int64_t get_score();
};
