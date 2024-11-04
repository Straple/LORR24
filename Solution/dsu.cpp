#include "dsu.hpp"

#include <numeric>

DSU::DSU(uint32_t size) {
    parent.resize(size);
    std::iota(parent.begin(), parent.end(), 0);
}

uint32_t DSU::get(uint32_t x) {
    if (parent[x] == x) {
        return x;
    } else {
        return parent[x] = get(parent[x]);
    }
}

void DSU::uni(uint32_t a, uint32_t b) {
    a = get(a);
    b = get(b);
    if (a != b) {
        parent[a] = b;
    }
}
