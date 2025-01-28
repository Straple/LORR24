#include <Objects/Environment/types.hpp>

MapType &get_map_type() {
    static MapType type = MapType::NONE;
    return type;
}

TestType &get_test_type() {
    static TestType type = TestType::NONE;
    return type;
}

