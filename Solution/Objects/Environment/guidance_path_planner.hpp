#pragma once

#include <cstdint>
#include <vector>

#include <Objects/Basic/time.hpp>
#include <settings.hpp>

static constexpr uint32_t GPP_DEPTH = 10;

// нужно уменьшить близорукость агентов
// эта штука планирует путь (возможно с конфликтами) некоторой длины
// при планировании учитываются агенты и штрафуются за проход по ним
// далее эти пути используются в PIBTS
class GuidancePathPlanner {
    // guidance_paths[r] = guidance path (nodes)
    std::vector<std::vector<uint32_t>> guidance_paths;

    // pos_to_robot[pos] = robot id or -1
    std::vector<uint32_t> pos_to_robot;

    void build(uint32_t r);

public:
    void update(uint32_t timestep, TimePoint end_time);
};
