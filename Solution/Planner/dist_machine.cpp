#include "dist_machine.hpp"

#include "../assert.hpp"

int DistMachine::get_dist(Position source, Position target, SharedEnvironment *env) {
    vector<Position> Q0, Q1;
    Q0.push_back(source);

    std::set<Position> visited;
    visited.insert(source);

    int d = 0;
    while (!Q0.empty() || !Q1.empty()) {
        if (Q0.empty()) {
            std::swap(Q0, Q1);
            d++;
        }

        Position p = Q0.back();
        Q0.pop_back();

        if (p == target) {
            return d;
        }

        visited.insert(p);

#define STEP(init)                                                  \
    {                                                               \
        Position q = (init);                                        \
        if (q.is_valide(env) && visited.find(q) == visited.end()) { \
            visited.insert(q);                                      \
            Q1.push_back(q);                                        \
        }                                                           \
    }

        STEP(p.move_forward(env));
        STEP(p.rotate());
        STEP(p.counter_rotate());

#undef STEP
    }

    ASSERT(false, "not found path");
    return -1;
}
