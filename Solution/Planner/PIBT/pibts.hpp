#pragma once

#include <Objects/Basic/randomizer.hpp>
#include <Planner/PIBT/pibt2.hpp>
#include <unordered_set>

// Priority Inheritance with BackTracking
// Each robot is assigned an action vector from the pool. Examples: FW, FW, W
// Solver mode
class PIBTS {
    const std::vector<Robot> &robots;

    TimePoint end_time;

    int64_t cur_score = 0;

    // used_edge[depth][edge] = robot id
    std::array<std::vector<uint32_t>, DEPTH> used_edge;

    // used_pos[depth][pos] = robot id
    std::array<std::vector<uint32_t>, DEPTH> used_pos;

    // desires[r] = запланированное действие робота
    std::vector<uint32_t> desires;

    struct State {
        int64_t cur_score = 0;

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

    [[nodiscard]] bool validate_path(uint32_t r, uint32_t desired) const;

    [[nodiscard]] bool is_free_path(uint32_t r) const;

    [[nodiscard]] bool is_free_path(uint32_t r, const State &state) const;

    [[nodiscard]] std::array<uint32_t, DEPTH> get_path(uint32_t r, uint32_t desired) const;

    [[nodiscard]] uint32_t get_used(uint32_t r) const;

    [[nodiscard]] uint32_t get_used(uint32_t r, const State &state) const;

    void update_score(uint32_t r, uint32_t finish_node, int64_t &cur_score, int sign) const;

    void add_path(uint32_t r);

    void add_path(uint32_t r, State &state) const;

    void remove_path(uint32_t r);

    void remove_path(uint32_t r, State &state) const;

    void flush_state(const State &state);

    bool try_build(uint32_t r, State &state, uint32_t &counter, Randomizer &rnd) const;

    bool try_build(uint32_t r, Randomizer &rnd);

    // return 0, if failed
    // return 1, if success+accepted
    // return 2, if success+not accepted
    uint32_t try_build2(uint32_t r, uint32_t &counter, Randomizer &rnd, int64_t old_score);

    bool try_build2(uint32_t r, Randomizer &rnd);

    bool build(uint32_t r, uint32_t depth, uint32_t &counter);

    bool build(uint32_t r);

public:
    static inline std::vector<Operation> actions;

    explicit PIBTS(const std::vector<Robot> &robots);

    std::vector<Action> solve(TimePoint end_time);
};
