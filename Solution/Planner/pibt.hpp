#pragma once

#include "../Objects/position.hpp"

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
        // чем выше, тем он важнее
        int priority = 0;
    };

    std::vector<Robot> robots;

    std::map<uint32_t, uint32_t> pos_to_robot;

    bool build(uint32_t r, int priority);

public:
    PIBT(const std::vector<Position>& robots_pos, const std::vector<int>& robots_target);

    std::vector<Action> solve();
};
