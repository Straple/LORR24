#pragma once

#include <array>

enum class MapType {
    RANDOM,
    GAME,
    CITY,
    WAREHOUSE,
    SORTATION,

    NONE,
};

MapType &get_map_type();

enum class PlannerType {
    PIBT,      // pibt
    PIBT_TF,   // pibt + trafficflow
    PIBT_LNS,  // pibt + lns
    P_PIBT_LNS,// parallel pibt + lns
    EPIBT,     // epibt
    EPIBT_LNS, // epibt + lns
    PEPIBT_LNS,// parallel epibt + lns
};

PlannerType &get_planner_type();
