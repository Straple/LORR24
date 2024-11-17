#include "environment.hpp"

#include "../Planner/Solver/planner_solver.hpp"
#include "../settings.hpp"
#include "assert.hpp"
#include "dsu.hpp"
#include "guidance_graph.hpp"
#include "linear_heap.hpp"

#include <thread>

void Environment::build_dists(uint32_t target) {
    Position source(target, 0);
    if (!source.is_valid()) {
        return;
    }

    dist_dp[target].resize(map.size());

    std::multiset<std::pair<int64_t, Position>> S;
    std::vector<std::array<bool, 4>> visited(map.size());

    for (int dir = 0; dir < 4; dir++) {
        source.dir = dir;
        S.insert({0, source});
    }

    while (!S.empty()) {
        ASSERT(!S.empty(), "is empty");
        auto [dist, p] = *S.begin();
        S.erase(S.begin());

        ASSERT(p.is_valid(), "p is invalid");
        if (visited[p.pos][p.dir]) {
            continue;
        }
        visited[p.pos][p.dir] = true;

        ASSERT(static_cast<uint32_t>(dist) == dist, "overflow");
        dist_dp[target][p.pos][(p.dir + 2) % 4] = dist;

        auto step = [&](Action action) {
            Position to = p.simulate_action(action);
            if (to.is_valid() && !visited[to.pos][to.dir]) {
                int64_t d = dist;
                d += get_gg().get(p.pos, p.dir, action);
                S.insert({d, to});
            }
        };

        step(Action::FW);
        step(Action::CR);
        step(Action::CCR);
    }
}

void Environment::build_dists() {
    //auto start = std::chrono::steady_clock::now();
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

    //std::cout << "build_dists time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() << "ms" << std::endl;
}

void Environment::build_robot_dist(uint32_t r) {
    if (robots[r].target == -1) {
        return;
    }
    robot_dists[r].resize(map.size());
    for (uint32_t pos = 0; pos < map.size(); pos++) {
        for (uint32_t dir = 0; dir < 4; dir++) {
            robot_dists[r][pos][dir] = static_cast<uint32_t>(1e18);
        }
    }

    LinearHeap<std::pair<int64_t, Position>> S;
    std::vector<std::array<bool, 4>> visited(map.size());

    for (int dir = 0; dir < 4; dir++) {
        Position source(robots[r].target, dir);
        S.push({0, source});
    }

    while (!S.empty()) {
        ASSERT(!S.empty(), "is empty");
        auto [dist, p] = S.top();
        S.pop();

        ASSERT(p.is_valid(), "p is invalid");
        if (visited[p.pos][p.dir]) {
            continue;
        }
        visited[p.pos][p.dir] = true;

        ASSERT(static_cast<uint32_t>(dist) == dist, "overflow");
        robot_dists[r][p.pos][(p.dir + 2) % 4] = dist;

        auto step = [&](Action action) {
            Position to = p.simulate_action(action);
            if (to.is_valid() && !visited[to.pos][to.dir]) {
                int64_t d = dist;
                d += get_gg().get(p.pos, p.dir, action);
                if (pos_to_robot[to.pos] != -1) {
                    d += 8000;
                }

                if (d < robot_dists[r][to.pos][(to.dir + 2) % 4]) {
                    //S.erase({robot_dists[r][to.pos][(to.dir + 2) % 4], to});
                    robot_dists[r][to.pos][(to.dir + 2) % 4] = d;
                    S.push({d, to});
                }
            }
        };

        step(Action::FW);
        step(Action::CR);
        step(Action::CCR);
    }
}

void Environment::build_robot_dists(std::chrono::steady_clock::time_point end_time) {
#ifdef ENABLE_DYNAMICS_DIST_MATRIX
    //auto start = std::chrono::steady_clock::now();
    ASSERT(robots.size() == get_agents_size(), "invalid sizes");
    robot_dists.resize(robots.size());

    auto do_work = [&](uint32_t thr) {
        for (uint32_t i = 0; i < (robots.size() + THREADS - 1) / THREADS && std::chrono::steady_clock::now() < end_time; i++) {
            ASSERT(last_finished_robot_dist.size() == THREADS, "invalid size");
            auto &r = last_finished_robot_dist[thr];
            if (r >= robots.size()) {
                break;
            }
            ASSERT(0 <= r && r < robots.size(), "invalid r");
            build_robot_dist(r);
            robots[r].predicted_dist = get_env().get_dist(r, robots[r].p);

            r += THREADS;
            if (r >= robots.size()) {
                r = thr;
            }
        }
    };

    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(do_work, thr);
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }

    //std::cout << "build_dists time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() << "ms" << std::endl;
#endif
}

void Environment::init(SharedEnvironment *env) {
    env_ptr = env;
    rows = env->rows;
    cols = env->cols;

    {
        //std::ifstream input("gg.txt");
        //input >> get_gg();
        get_gg().graph.resize(get_size());
        for (uint32_t pos = 0; pos < get_size(); pos++) {
            for (uint32_t dir = 0; dir < 4; dir++) {
                for (uint32_t action = 0; action < 4; action++) {
                    get_gg().graph[pos][dir][action] = 10000;
                }
            }
        }
    }

    ASSERT(env->map.size() == cols * rows, "invalid env sizes: " + std::to_string(env->map.size()) + " != " +
                                                   std::to_string(cols) + " * " + std::to_string(rows));
    map.resize(env->map.size());
    for (uint32_t pos = 0; pos < map.size(); pos++) {
        map[pos] = env->map[pos] == 0;
    }

    last_finished_robot_dist.resize(THREADS);
    for (uint32_t i = 0; i < THREADS; i++) {
        last_finished_robot_dist[i] = i;
    }

#ifdef ENABLE_DIST_MATRIX
    build_dists();
#endif
}

void Environment::build_robots() {
    pos_to_robot.assign(map.size(), -1);
    robots.resize(env_ptr->num_of_agents);
    //robot_dists.resize(robots.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        robots[r].p = Position(env_ptr->curr_states[r].location, env_ptr->curr_states[r].orientation);
        robots[r].target = -1;
        robots[r].task = -1;
        robots[r].predicted_dist = 1e17;
        ASSERT(pos_to_robot[robots[r].p.pos] == -1, "already used");
        pos_to_robot[robots[r].p.pos] = r;

        //robot_dists[r].resize(map.size());
    }

    for (uint32_t t = 0; t < env_ptr->task_pool.size(); t++) {
        auto &task = env_ptr->task_pool[t];
        int r = task.agent_assigned;
        if (r != -1) {
            robots[r].task = t;
            robots[r].target = task.get_next_loc();
            ASSERT(0 <= robots[r].target && robots[r].target < get_size(), "invalid target");
            ASSERT(task.idx_next_loc < task.locations.size(), "why?");
            robots[r].predicted_dist = get_env().get_dist(robots[r].p, robots[r].target);
        }
    }
}

[[nodiscard]] Environment::Robot Environment::get_robot(uint32_t r) const {
    return robots[r];
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

int Environment::get_agents_size() const {
    return env_ptr->num_of_agents;
}

bool Environment::is_free(uint32_t pos) const {
    ASSERT(pos < map.size(), "invalid pos: " + std::to_string(pos));
    return map[pos];
}

int64_t Environment::get_dist(Position source, int target) const {
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

int64_t Environment::get_dist(uint32_t r, Position source) const {
    ASSERT(r < robots.size(), "invalid r");
    ASSERT(source.is_valid(), "invalid source");
    ASSERT(0 <= source.dir && source.dir < 4, "invalid dir");

#ifdef ENABLE_DYNAMICS_DIST_MATRIX
    if (robots[r].target == -1) {
        return 0;
    }
    ASSERT(r < robot_dists.size(), "invalid robot dists");
    if (robot_dists[r].empty()) {
        return get_dist(source, robots[r].target);
    }
    ASSERT(source.pos < robot_dists[r].size(), "invalid pos");
    return robot_dists[r][source.pos][source.dir];
#else
    return get_dist(source, robots[r].target);
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
