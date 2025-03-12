#pragma once

#include <Objects/Basic/position.hpp>
#include <Objects/Basic/time.hpp>
#include <Objects/Environment/operations.hpp>
#include <Objects/Environment/robot_handler.hpp>

// Enhanced Priority Inheritance with BackTracking
class EPIBT {
    TimePoint end_time;

    const std::vector<Robot> &robots;

    std::vector<uint32_t> desires;

    std::vector<uint32_t> order;

    // robot_desires[r] = { desired }
    std::vector<std::vector<uint32_t>> robot_desires;

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

/*
Actions:
F: 153
R: 95
C: 81
W: 71
N: 0
Total action:
F: 174269
R: 94064
C: 57231
W: 74436
N: 0
Entry time: 7.96152ms
Total time: 12.9241s
[2025-03-12 21:51:58.263531] [0x00007f5a787f9740] [info]    [timestep=999] planner returns
[2025-03-12 21:51:58.263963] [0x00007f5a787f9740] [info]    [timestep=1000] Agent 262 finishes task 3762
[2025-03-12 21:51:58.263991] [0x00007f5a787f9740] [info]    [timestep=1000] Agent 354 finishes task 3347
[2025-03-12 21:51:58.263997] [0x00007f5a787f9740] [info]    Task 4121 is revealed
[2025-03-12 21:51:58.264000] [0x00007f5a787f9740] [info]    Task 4122 is revealed
*/