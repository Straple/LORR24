#pragma once

#include "SharedEnv.h"
#include "position.hpp"

#include <cstdint>
#include <vector>
#include <array>

#define TRIVIAL_DIST_HEURISTIC

static constexpr uint32_t THREADS = 4;

class Environment {
    int rows = 0, cols = 0;

    // map[pos] = true if this pos is free
    // otherwise: false
    std::vector<bool> map;

    //// dist_dp[target][source][dir] = dist from (source, dir) -> target
    std::vector<std::vector<std::array<uint16_t, 4> > > dist_dp;

    void build_dists(uint32_t target);

    void build_dists();

public:

    void init(SharedEnvironment *env);

    [[nodiscard]] int get_rows() const;

    [[nodiscard]] int get_cols() const;

    [[nodiscard]] int get_size() const;

    [[nodiscard]] bool is_free(uint32_t pos) const;

    [[nodiscard]] int get_dist(Position p, uint32_t target) const;


};

Environment &get_env();
