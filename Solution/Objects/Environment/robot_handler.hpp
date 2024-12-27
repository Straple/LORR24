#pragma once

#include <SharedEnv.h>
#include <cstdint>
#include <vector>

class RobotsHandler {
public:
    struct Robot {
        uint32_t node = 0;    // start node from graph
        uint32_t target = 0;  // target pos from map
        uint32_t priority = 0;// like dist to target
    };

private:
    std::vector<Robot> robots;

public:
    RobotsHandler() = default;
    explicit RobotsHandler(SharedEnvironment &env);

    [[nodiscard]] const Robot &get_robot(uint32_t r) const;

    [[nodiscard]] uint32_t size() const;
};

RobotsHandler &get_robots_handler();
