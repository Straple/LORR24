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
    std::vector<std::vector<std::array<uint16_t, 4>>> dist_dp;

    void build_dists(uint32_t target);

    void build_dists();

    std::vector<int> map_major;

public:
    struct Robot {
        Position p;
        int task = -1;
        int target = -1;
        int64_t predicted_dist = 0;
    };

private:
    std::vector<Robot> robots;

    // robot_dists[r][source][dir]
    std::vector<std::vector<std::array<uint32_t, 4>>> robot_dists;

    std::vector<int> pos_to_robot;

    std::vector<uint32_t> last_finished_robot_dist;

    void build_robot_dist(uint32_t r);

public:
    void init(SharedEnvironment *env);

    void build_robots();

    void build_robot_dists(std::chrono::steady_clock::time_point end_time);

    [[nodiscard]] Robot get_robot(uint32_t r) const;

    [[nodiscard]] int get_rows() const;

    [[nodiscard]] int get_cols() const;

    [[nodiscard]] int get_size() const;

    [[nodiscard]] int get_agents_size() const;

    // no '#'
    [[nodiscard]] bool is_free(uint32_t pos) const;

    // p -> target
    [[nodiscard]] int64_t get_dist(Position source, int target) const;

    // (robot, source)
    [[nodiscard]] int64_t get_dist(uint32_t r, Position source) const;

    // and build map_major for get_major()
    std::vector<std::vector<int>> split_robots(SharedEnvironment *env);

    [[nodiscard]] int get_major(uint32_t pos) const;

    [[nodiscard]] SharedEnvironment &get_shared_env() const;
};

Environment &get_env();
