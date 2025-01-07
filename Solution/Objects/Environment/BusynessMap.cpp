#include "BusynessMap.hpp"


BusynessMap::BusynessMap(const Map& full_map, int32_t compress_factor_): compress_factor(compress_factor_){
        size_t cols_compressed = full_map.get_cols()/compress_factor;
        size_t rows_compressed = full_map.get_rows()/compress_factor;
        std::vector<bool> compressed(cols_compressed*rows_compressed,true);
        uint32_t cur_pos = 0;
        for (uint32_t y = 0; y < full_map.get_rows(); y++){
            for (uint32_t x = 0; x < full_map.get_cols(); x++){
                cur_pos++;
                if (full_map.is_free(cur_pos)){
                    uint32_t compressed_pos = (y/compress_factor)*cols_compressed + (x/compress_factor);
                    compressed[compressed_pos] = false;
                }

            }
        }
        mp = Map(compressed, cols_compressed, rows_compressed);
        Graph graph(mp, get_gg());
        hm = HeuristicMatrix(graph);
}

BusynessMap &get_busyness_map(){
    static BusynessMap mp;
    return mp;
}