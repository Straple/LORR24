#pragma once

#include "SharedEnv.h"
#include "position.hpp"

#include <array>
#include <cstdint>
#include <vector>

class Environment {
    SharedEnvironment *env_ptr = nullptr;

    int rows = 0, cols = 0;

    // map[pos] = true if this pos is free
    // otherwise: false
    std::vector<bool> map;

    // dist_dp[target][source][dir] = dist from (source, dir) -> target
    std::vector<std::vector<std::array<uint32_t, 4>>> dist_dp;

    void build_dists(uint32_t target);

    void build_dists();

    std::vector<int> map_major;

public:
    void init(SharedEnvironment *env);

    [[nodiscard]] int get_rows() const;

    [[nodiscard]] int get_cols() const;

    [[nodiscard]] int get_size() const;

    // no '#'
    [[nodiscard]] bool is_free(uint32_t pos) const;

    // p -> target
    [[nodiscard]] int64_t get_dist(Position source, int target) const;

    // and build map_major for get_major()
    std::vector<std::vector<int>> split_robots(SharedEnvironment *env);

    [[nodiscard]] int get_major(uint32_t pos) const;

    [[nodiscard]] SharedEnvironment &get_shared_env() const;
};

Environment &get_env();
