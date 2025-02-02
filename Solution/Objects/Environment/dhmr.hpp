#pragma once

#include <Objects/Basic/time.hpp>

#include <vector>
#include <cstdint>

// Dynamic Heuristic Matrix for Robots
class DHMR {
    // [r][desired]
    std::vector<std::vector<uint32_t>> matrix;

    // pos_to_robot[pos] = robot id or -1
    std::vector<uint32_t> pos_to_robot;

    [[nodiscard]] uint32_t get_triv(uint32_t r, uint32_t desired, uint32_t robot_node, uint32_t target, std::vector<uint32_t>& visited, uint32_t &visited_counter, std::vector<uint32_t> &dp) const;

    // тут немного хуже ответ почему-то (+2-6)
    [[nodiscard]] uint32_t get(uint32_t r, uint32_t desired, uint32_t robot_node, uint32_t target, std::vector<uint32_t>& visited, uint32_t &visited_counter, std::vector<uint32_t> &dp) const;

public:
    void update(uint32_t timestep, TimePoint end_time);

    [[nodiscard]] uint32_t get(uint32_t r, uint32_t desired) const;
};

DHMR &get_dhmr();
