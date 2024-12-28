#pragma once

#include <Objects/Basic/position.hpp>
#include <Objects/Basic/time.hpp>

static constexpr inline uint32_t DEPTH = 3;

using Operation = std::array<Action, DEPTH>;

class BuilderActions {

    std::vector<Operation> pool;

    void generate(Operation &op, uint32_t i);

public:
    std::vector<Operation> get();
};

class PIBT2 {

    static inline std::vector<Operation> actions = BuilderActions().get();

    // used_edge[depth][edge] = robot id
    std::array<std::vector<uint32_t>, DEPTH> used_edge;

    // used_pos[depth][pos] = robot id
    std::array<std::vector<uint32_t>, DEPTH> used_pos;

    struct Robot {
        uint32_t start_node = 0;

        // действие, которое он хочет
        uint32_t desired = 0;
    };

    std::vector<Robot> robots;

    TimePoint end_time;

    uint32_t counter_call_build = 0;

    bool finish_time = false;

    [[nodiscard]] bool validate_path(uint32_t r) const;

    [[nodiscard]] bool check_path(uint32_t r) const;

    [[nodiscard]] std::array<uint32_t, DEPTH> get_path(uint32_t r) const;

    [[nodiscard]] uint32_t get_used(uint32_t r) const;

    void add_path(uint32_t r);

    void remove_path(uint32_t r);

    bool build(uint32_t r, uint32_t depth);

public:
    PIBT2();

    std::vector<Action> solve(const std::vector<uint32_t> &order, const TimePoint end_time);
};

// 4255 -> 8217 -> 9992 -> 12412 -> 14427 -> 14427 -> 14073