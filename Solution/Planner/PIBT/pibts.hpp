#pragma once

#include <Objects/Basic/randomizer.hpp>
#include <Planner/PIBT/pibt2.hpp>
#include <unordered_set>

// -i ./example_problems/random.domain/random_32_32_20_300.json -o test.json -s 1000 -t 40 -p 100000000 --unique_id 0
// 61.6601s
// 4245 -> 4449

// -i ./example_problems/warehouse.domain/warehouse_large_5000.json -o test.json -s 1000 -t 70 -p 100000000
// 65.7547s
// PIBTS: 13509
// PIBTS+brezhart: 12244
// PIBTS+egor:     12339
// their: 10338
// 669.072s
// PIBTS: 141219
// their: 114397
// PIBT2: 116357

// -i ./example_problems/random.domain/random_32_32_20_300.json -o test.json -s 10000 -t 10000 -p 100000000
// without PIBTS
// score: 28611
// timer: 15.0559s
//
// PIBTS_STEPS = 500
// score: 35969 -> 37049 -> 37914
// timer: 50.202s

// Priority Inheritance with BackTracking
// Each robot is assigned an action vector from the pool. Examples: FW, FW, W
// Solver mode
class PIBTS {

    double temp = 1;

    double cur_score = 0;

    const std::vector<Robot> &robots;

    TimePoint end_time;

    // used_edge[depth][edge] = robot id
    std::array<std::vector<uint32_t>, DEPTH> used_edge;

    // used_pos[depth][pos] = robot id
    std::array<std::vector<uint32_t>, DEPTH> used_pos;

    // desires[r] = запланированное действие робота
    std::vector<uint32_t> desires;

    struct State {
        double cur_score = 0;

        // used_edge[depth][edge] = robot id
        std::array<std::unordered_map<uint32_t, uint32_t>, DEPTH> used_edge;

        // used_pos[depth][pos] = robot id
        std::array<std::unordered_map<uint32_t, uint32_t>, DEPTH> used_pos;

        // desires[r] = запланированное действие робота
        std::unordered_map<uint32_t, uint32_t> desires;
    };

    std::vector<uint32_t> order;

    std::vector<uint32_t> weight;

    std::vector<uint32_t> visited;
    uint32_t visited_counter = 1;

    Randomizer rnd;

    double old_score = 0;

    [[nodiscard]] bool validate_path(uint32_t r, uint32_t desired) const;

    [[nodiscard]] bool is_free_path(uint32_t r) const;

    [[nodiscard]] bool is_free_path(uint32_t r, const State &state) const;

    [[nodiscard]] EPath get_path(uint32_t r, uint32_t desired) const;

    [[nodiscard]] uint32_t get_used(uint32_t r) const;

    [[nodiscard]] uint32_t get_used(uint32_t r, const State &state) const;

    void update_score(uint32_t r, uint32_t finish_node, double &cur_score, int sign) const;

    void add_path(uint32_t r);

    void add_path(uint32_t r, State &state) const;

    void remove_path(uint32_t r);

    void remove_path(uint32_t r, State &state) const;

    void flush_state(const State &state);

    bool try_build_state(uint32_t r, State &state, uint32_t &counter, Randomizer &rnd) const;

    bool try_build_state(uint32_t r, Randomizer &rnd);

    // return 0, if failed
    // return 1, if success+accepted
    // return 2, if success+not accepted
    uint32_t try_build(uint32_t r, uint32_t &counter, uint32_t depth);

    bool try_build(uint32_t r);

    bool build(uint32_t r, uint32_t depth, uint32_t &counter);

    bool build(uint32_t r);

public:
    explicit PIBTS(const std::vector<Robot> &robots);

    std::vector<Action> solve(TimePoint end_time, uint64_t seed);

    [[nodiscard]] double get_score() const;
};
