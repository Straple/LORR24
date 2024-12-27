#pragma once

#include <Objects/Basic/assert.hpp>

#include <chrono>
#include <iostream>

using TimePoint = std::chrono::steady_clock::time_point;
using Milliseconds = std::chrono::milliseconds;
using Nanoseconds = std::chrono::nanoseconds;

TimePoint get_now();

class Timer {
    TimePoint start;

public:
    Timer();

    [[nodiscard]] uint64_t get_ms() const;

    [[nodiscard]] uint64_t get_ns() const;

    void reset();
};

std::ostream &operator<<(std::ostream &output, const Timer &time);
