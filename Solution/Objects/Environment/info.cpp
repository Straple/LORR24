#include <Objects/Environment/info.hpp>

MapType &get_map_type() {
    static MapType type = MapType::NONE;
    return type;
}

TestType &get_test_type() {
    static TestType type = TestType::NONE;
    return type;
}

static std::array<TestInfo, static_cast<uint32_t>(TestType::NONE)> test_info = {
        // TODO: CITY!
        TestInfo{0},    // CITY_1
        TestInfo{0},    // CITY_2
        TestInfo{5000}, // GAME
        TestInfo{600}, // RANDOM_1
        TestInfo{600}, // RANDOM_2
        TestInfo{800}, // RANDOM_3
        TestInfo{1000}, // RANDOM_4
        TestInfo{2000}, // RANDOM_5
        TestInfo{5000}, // SORTATION
        TestInfo{5000}, // WAREHOUSE
};

TestInfo get_test_info() {
    return test_info[static_cast<uint32_t>(get_test_type())];
}
