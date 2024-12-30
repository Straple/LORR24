#pragma once

#include <Objects/Basic/position.hpp>
#include <Objects/Basic/time.hpp>
#include <Objects/Environment/robot_handler.hpp>
#include <unordered_map>

static constexpr inline uint32_t DEPTH = 3;

using Operation = std::array<Action, DEPTH>;

class BuilderActions {

    std::vector<Operation> pool;

    void generate(Operation &op, uint32_t i);

public:
    std::vector<Operation> get();
};

// Priority Inheritance with BackTracking
// Each robot is assigned an action vector from the pool. Examples: FW, FW, W
class PIBT2 {
    // used_edge[depth][edge] = robot id
    std::array<std::vector<uint32_t>, DEPTH> used_edge;

    // used_pos[depth][pos] = robot id
    std::array<std::vector<uint32_t>, DEPTH> used_pos;

    // [r][edge] = weight
    std::vector<std::unordered_map<uint32_t, uint32_t>> weights;

    std::vector<Robot> robots;

    // действие, которое он хочет
    std::vector<uint32_t> desires;

    TimePoint end_time;

    uint32_t counter_call_build = 0;

    bool finish_time = false;

    [[nodiscard]] bool validate_path(uint32_t r) const;

    [[nodiscard]] bool check_path(uint32_t r) const;

    [[nodiscard]] std::array<uint32_t, DEPTH> get_path(uint32_t r) const;

    [[nodiscard]] uint32_t get_path_weight(uint32_t r) const;

    [[nodiscard]] uint32_t get_used(uint32_t r) const;

    void add_path(uint32_t r);

    void remove_path(uint32_t r);

    bool build(uint32_t r, uint32_t depth, uint32_t &counter);

public:
    static inline std::vector<Operation> actions;

    PIBT2(std::vector<Robot> robots, std::vector<std::unordered_map<uint32_t, uint32_t>> weights);

    std::vector<Action> solve(const std::vector<uint32_t> &order, const TimePoint end_time);
};
