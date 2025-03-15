#include <Objects/Environment/info.hpp>

MapType &get_map_type() {
    static MapType type = MapType::NONE;
    return type;
}

PlannerType &get_planner_type() {
    static PlannerType type = PlannerType::NONE;
    return type;
}
