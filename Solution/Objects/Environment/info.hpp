#pragma once

#include <array>
#include <cstdint>

enum class MapType {
    RANDOM,
    GAME,
    CITY,
    WAREHOUSE,
    SORTATION,

    NONE,
};

MapType &get_map_type();

enum class TestType {
    CITY_1,
    CITY_2,
    GAME,
    RANDOM_1,
    RANDOM_2,
    RANDOM_3,
    RANDOM_4,
    RANDOM_5,
    SORTATION,
    WAREHOUSE,

    NONE,
};

TestType &get_test_type();

struct TestInfo {
    uint32_t steps_num = 0;
    uint32_t max_task_assigned = 0;
};

TestInfo get_test_info();

enum class PlannerType {
    PIBT,      // pibt
    PIBT_LNS,  // pibt + lns
    P_PIBT_LNS,// parallel pibt + lns
    EPIBT,     // epibt
    EPIBT_LNS, // epibt + lns
    PEPIBT_LNS,// parallel epibt + lns
};

PlannerType &get_planner_type();
