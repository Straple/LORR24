#pragma once

#include <SharedEnv.h>
#include <cstdint>
#include <vector>

struct Robot {
    uint32_t prev_node = 0;
    uint32_t node = 0;    // start node from graph
    uint32_t prev_target = 0;
    uint32_t target = 0;  // target pos from map
    double priority = 0;
    double penalty = 1;
};

class RobotsHandler {
    std::vector<Robot> robots;

public:
    RobotsHandler() = default;

    explicit RobotsHandler(uint32_t agents_num);

    void update(const SharedEnvironment &env);

    [[nodiscard]] const Robot &get_robot(uint32_t r) const;

    [[nodiscard]] const std::vector<Robot> &get_robots() const;

    [[nodiscard]] uint32_t size() const;
};

RobotsHandler &get_robots_handler();
