#pragma once

#include "graph.hpp"
#include "heuristic_matrix.hpp"
#include <cstdint>
#include <vector>

class BusynessMap {
private:
    Map mp;
    HeuristicMatrix hm;
    int32_t compress_factor = 5;
public:
    BusynessMap() = default;
    BusynessMap(const Map& full_map, int32_t compress_factor_ = 5);
    uint32_t GetCompressedPos(size_t x, size_t y){
        return 3;
    }
};

BusynessMap &get_busyness_map();