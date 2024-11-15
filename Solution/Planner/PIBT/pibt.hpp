#pragma once

#include "../../Objects/position.hpp"

// 1000: 1604 -> 1623 -> 1635
// 10'000: 12131 -> 14132

// Priority Inheritance with BackTracking
class PIBT {
    struct Robot {
        Position p;

        // куда мы хотим
        // -1 -- не определено
        // иначе это направление для forward
        int dir = -1;

        // то куда мы хотим попасть
        int target = -1;

        // приоритет робота
        // чем ниже, тем он важнее
        int priority = 0;
    };

    std::vector<Robot> robots;

    std::unordered_map<uint32_t, uint32_t> pos_to_robot;

    bool build(uint32_t r, int banned_direction = -1);

public:
    PIBT(const std::vector<Position>& robots_pos, const std::vector<int>& robots_target, const std::vector<int>& robot_priority);

    std::vector<Action> solve();
};
