#pragma once

#include "solution_info.hpp"
#include "SharedEnv.h"

#include <vector>
#include <cstdint>

class GlobalDP {
    // [d][pos]
    std::vector<std::vector<uint32_t>> map_robots_cnt;

    // [d][pos]
    std::vector<std::vector<uint32_t>> map_edge_robots_cnt_gor, map_edge_robots_cnt_ver;

    // pos_to_robot[pos] = robot id or -1
    std::vector<int> pos_to_robot;

public:

    [[nodiscard]] int get_robot(int pos) const;

    void change_map_robots_cnt(int d, int pos, int val, SolutionInfo &info);

    void change_map_edge_robots_cnt(int d, int pos, int to, int val, SolutionInfo &info);

    void init(SharedEnvironment* env);
};

GlobalDP &get_global_dp();