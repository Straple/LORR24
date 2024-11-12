#pragma once

#include "../settings.hpp"

#include <array>
#include <cstdint>
#include <iostream>

struct SolutionInfo {

    // количество столкновений у роботов
    // уменьшаем это значение
    // если оно не ноль, то это плохо
    std::array<uint32_t, PLANNER_DEPTH> collision_count{0};

    // берем каждого робота
    // смотрим на сколько он улучшает свое расстояние до таргета в среднем за PLANNER_DEPTH шагов
    // берем такое среднее значение по всем роботам
    // увеличиваем это значение
    std::array<int64_t, PLANNER_DEPTH> sum_dist_change{0};

    std::array<uint32_t, PLANNER_DEPTH> count_forward{0};
};

bool operator==(const SolutionInfo &lhs, const SolutionInfo &rhs);

bool operator!=(const SolutionInfo &lhs, const SolutionInfo &rhs);

SolutionInfo operator+(SolutionInfo lhs, const SolutionInfo &rhs);

std::ostream &operator<<(std::ostream &output, const SolutionInfo &info);
