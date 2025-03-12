#pragma once

#include <Objects/Basic/position.hpp>
#include <Objects/Basic/time.hpp>
#include <Objects/Environment/robot_handler.hpp>
#include <Objects/Environment/operations.hpp>

// Enhanced Priority Inheritance with BackTracking
class EPIBT {
    TimePoint end_time;

    const std::vector<Robot> &robots;

    std::vector<uint32_t> desires;

    // used_edge[edge][depth] = robot id
    std::vector<std::array<uint32_t, DEPTH>> used_edge;

    // used_pos[pos][depth] = robot id
    std::vector<std::array<uint32_t, DEPTH>> used_pos;

    [[nodiscard]] bool validate_path(uint32_t r, uint32_t desired) const;

    [[nodiscard]] bool is_free_path(uint32_t r) const;

    [[nodiscard]] const EPath &get_path(uint32_t r, uint32_t desired) const;

    [[nodiscard]] uint32_t get_used(uint32_t r) const;

    [[nodiscard]] int64_t get_smart_dist_IMPL(uint32_t r, uint32_t desired) const;

    void add_path(uint32_t r);

    void remove_path(uint32_t r);

    bool build(uint32_t r, uint32_t depth, uint32_t &counter);

public:
    EPIBT(const std::vector<Robot> &robots, TimePoint end_time);

    void solve();

    [[nodiscard]] std::vector<Action> get_actions() const;
};
