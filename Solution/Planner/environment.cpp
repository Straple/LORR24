#include "environment.hpp"

#include "../assert.hpp"

#include <thread>

void Environment::build_dists(uint32_t target) {
    Position source(target, 0);
    if (!source.is_valid()) {
        return;
    }

    dist_dp[target].resize(map.size());

    vector<Position> Q0, Q1;
    std::vector<std::array<bool, 4>> visited(map.size());
    {
        for (int dir = 0; dir < 4; dir++) {
            source.dir = dir;
            Q0.push_back(source);
            visited[source.pos][source.dir] = true;
        }
    }

    int d = 0;
    while (!Q0.empty() || !Q1.empty()) {
        if (Q0.empty()) {
            std::swap(Q0, Q1);
            d++;
        }

        ASSERT(!Q0.empty(), "Q0 is empty");
        Position p = Q0.back();
        Q0.pop_back();

        dist_dp[target][p.pos][(p.dir + 2) % 4] = d;

        ASSERT(p.is_valid(), "p is invalid");

#define STEP(init)                                                  \
    {                                                               \
        Position to = (init);                                       \
        if (to.is_valid() && !visited[to.pos][to.dir]) {            \
            visited[to.pos][to.dir] = true;                         \
            Q1.push_back(to);                                       \
        }                                                           \
    }

        STEP(p.move_forward());
        STEP(p.rotate());
        STEP(p.counter_rotate());

#undef STEP
    }
}

void Environment::build_dists() {
    auto start = std::chrono::steady_clock::now();
    dist_dp.resize(map.size());

    auto do_work = [&](uint32_t thr) {
        for (uint32_t target = thr; target < map.size(); target += THREADS) {
            build_dists(target);
        }
    };

    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(do_work, thr);
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }
}

void Environment::init(SharedEnvironment *env) {
    rows = env->rows;
    cols = env->cols;

    ASSERT(env->map.size() == cols * rows, "invalid env sizes");
    map.resize(env->map.size());
    for (uint32_t pos = 0; pos < map.size(); pos++) {
        map[pos] = env->map[pos] == 0;
    }

#ifndef TRIVIAL_DIST_HEURISTIC
    build_dists();
#endif
}

int Environment::get_rows() const {
    return rows;
}

int Environment::get_cols() const {
    return cols;
}

int Environment::get_size() const {
    return rows * cols;
}

bool Environment::is_free(uint32_t pos) const {
    ASSERT(pos < map.size(), "invalid pos");
    return map[pos];
}

int Environment::get_dist(Position p, int target) const {
    if (target == -1) {
        return 0;
    }
    ASSERT(p.is_valid(), "invalid p");
    ASSERT(0 <= target && target < map.size(), "invalid target: " + std::to_string(target));
    ASSERT(0 <= p.pos && p.pos < map.size(), "invalid pos: " + std::to_string(p.pos));
    ASSERT(0 <= p.dir && p.dir < 4, "invalid dir");

#ifdef TRIVIAL_DIST_HEURISTIC
    Position to(target, 0);
    return std::abs(p.x - to.x) + std::abs(p.y - to.y);
#else
    return dist_dp[target][p.pos][p.dir];
#endif
}

Environment &get_env() {
    static Environment env;
    return env;
}
