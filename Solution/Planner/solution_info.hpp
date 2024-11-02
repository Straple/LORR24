#pragma once

#include <cstdint>

struct SolutionInfo {

    // количество столкновений у роботов
    // уменьшаем это значение
    // если оно не ноль, то это плохо
    uint32_t collision_count = 0;

    // берем каждого робота
    // смотрим на сколько он улучшает свое расстояние до таргета в среднем за PLANNER_DEPTH шагов
    // берем такое среднее значение по всем роботам
    // увеличиваем это значение
    long long sum_dist_change = 0;

    uint32_t count_forward = 0;
};
