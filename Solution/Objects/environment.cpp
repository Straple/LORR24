#include "environment.hpp"

#include "../Planner/Solver/planner_solver.hpp"
#include "../settings.hpp"
#include "assert.hpp"
#include "dsu.hpp"

#include <thread>

void Environment::build_dists(uint32_t target) {
    Position source(target, 0);
    if (!source.is_valid()) {
        return;
    }

    dist_dp[target].resize(map.size());

    vector<Position> Q0, Q1;
    std::vector<std::array<bool, 4>> visited(map.size());

    for (int dir = 0; dir < 4; dir++) {
        source.dir = dir;
        Q0.push_back(source);
        visited[source.pos][source.dir] = true;
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

#define STEP(init)                                       \
    {                                                    \
        Position to = (init);                            \
        if (to.is_valid() && !visited[to.pos][to.dir]) { \
            visited[to.pos][to.dir] = true;              \
            Q1.push_back(to);                            \
        }                                                \
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
    env_ptr = env;
    rows = env->rows;
    cols = env->cols;

    ASSERT(env->map.size() == cols * rows, "invalid env sizes: " + std::to_string(env->map.size()) + " != " +
                                                   std::to_string(cols) + " * " + std::to_string(rows));
    map.resize(env->map.size());
    for (uint32_t pos = 0; pos < map.size(); pos++) {
        map[pos] = env->map[pos] == 0;
    }

#ifdef ENABLE_DIST_MATRIX
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
    ASSERT(pos < map.size(), "invalid pos: " + std::to_string(pos));
    return map[pos];
}

int Environment::get_dist(Position source, int target) const {
    if (target == -1) {
        return 0;
    }
    ASSERT(source.is_valid(), "invalid p");
    ASSERT(0 <= target && target < map.size(), "invalid target: " + std::to_string(target));
    ASSERT(0 <= source.pos && source.pos < map.size(), "invalid pos: " + std::to_string(source.pos));
    ASSERT(0 <= source.dir && source.dir < 4, "invalid dir: " + std::to_string(source.dir));

#ifndef ENABLE_DIST_MATRIX
    Position to(target, 0);
    return std::abs(source.x - to.x) + std::abs(source.y - to.y);
#else
    return dist_dp[target][source.pos][source.dir];
#endif
}

std::vector<std::vector<int>> Environment::split_robots(SharedEnvironment *env) {
    DSU dsu(env->num_of_agents);

    map_major.assign(map.size(), -1);

    std::vector<std::pair<Position, int>> Q0, Q1;
    std::vector<std::set<Position>> visited(env->num_of_agents);

    for (uint32_t r = 0; r < env->num_of_agents; r++) {
        Position p(env->curr_states[r].location, env->curr_states[r].orientation);
        Q0.emplace_back(p, r);
        visited[r].insert(p);
    }

    Randomizer rnd(std::chrono::steady_clock::now().time_since_epoch().count());
    std::shuffle(Q0.begin(), Q0.end(), rnd.generator);

    int d = 0;
    while (!Q0.empty() || !Q1.empty()) {
        if (Q0.empty()) {
            std::swap(Q0, Q1);
            std::shuffle(Q0.begin(), Q0.end(), rnd.generator);
            d++;
        }

        ASSERT(d <= PLANNER_DEPTH, "invalid d");

        auto [p, r] = Q0.back();
        Q0.pop_back();

        ASSERT(p.is_valid(), "p is invalid");

        // paint
        {
            if (map_major[p.pos] == -1) {
                map_major[p.pos] = r;
            } else {
                uint32_t a = dsu.get(map_major[p.pos]);
                uint32_t b = dsu.get(r);
                if (a != b && dsu.get_size(a) + dsu.get_size(b) <= SPLIT_ROBOTS_BOUND) {
                    dsu.uni(map_major[p.pos], r);
                }
            }
        }

        if (d == PLANNER_DEPTH) {
            continue;
        }

#define STEP(init)                                    \
    {                                                 \
        Position to = (init);                         \
        if (to.is_valid() && !visited[r].count(to)) { \
            visited[r].insert(to);                    \
            Q1.emplace_back(to, r);                   \
        }                                             \
    }

        STEP(p.move_forward());
        STEP(p.rotate());
        STEP(p.counter_rotate());

#undef STEP
    }

    std::set<pair<uint32_t, int>> S;
    for (uint32_t r = 0; r < env->num_of_agents; r++) {
        S.insert({dsu.get_size(r), dsu.get(r)});
    }
    while (S.size() >= 2) {
        if (S.begin()->first + (++S.begin())->first <= SPLIT_ROBOTS_BOUND) {
            dsu.uni(S.begin()->second, (++S.begin())->second);
            int r = dsu.get(S.begin()->second);
            uint32_t sz = dsu.get_size(r);
            S.erase(S.begin());
            S.erase(S.begin());
            S.insert({sz, r});
        } else {
            break;
        }
    }

    std::vector<std::vector<int>> robots(env->num_of_agents);
    for (uint32_t r = 0; r < env->num_of_agents; r++) {
        robots[dsu.get(r)].push_back(r);
    }
    for (uint32_t r = 0; r < robots.size(); r++) {
        if (robots[r].empty()) {
            std::swap(robots[r], robots.back());
            robots.pop_back();
            r--;
        }
    }

    for (uint32_t p = 0; p < map.size(); p++) {
        if (map_major[p] != -1) {
            map_major[p] = dsu.get(map_major[p]);
        }
    }
    return robots;
}

int Environment::get_major(uint32_t pos) const {
    ASSERT(pos < map_major.size(), "invalid pos: " + std::to_string(pos) + "/" + std::to_string(map_major.size()));
    return map_major[pos];
}

SharedEnvironment &Environment::get_shared_env() const {
    ASSERT(env_ptr != nullptr, "is nullptr");
    return *env_ptr;
}

Environment &get_env() {
    static Environment env;
    return env;
}
