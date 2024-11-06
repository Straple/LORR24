#pragma once

#include <vector>
#include <cstdint>

class DSU {
    std::vector<uint32_t> parent;
public:

    DSU(uint32_t size);

    uint32_t get(uint32_t x);

    void uni(uint32_t a, uint32_t b);
};
