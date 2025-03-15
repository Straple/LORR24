#include <Objects/Environment/guidance_map.hpp>

#include <Objects/Basic/assert.hpp>

void GuidanceMap::set_random(const Map &map) {
    std::ifstream input("Solution/Data/guidance_map_random.txt");
    input >> *this;
}

void GuidanceMap::set_city(const Map &map) {
}

void GuidanceMap::set_game(const Map &map) {
}

void GuidanceMap::set_warehouse(const Map &map) {
}

void GuidanceMap::set_sortation(const Map &map) {
}

void GuidanceMap::overlay(const std::vector<std::string> &image, uint32_t x, uint32_t y) {
    for (uint32_t dx = 0; dx < image.size(); dx++) {
        for (uint32_t dy = 0; dy < image[dx].size(); dy++) {
            ASSERT(x + dx < desired.size(), "invalid x");
            ASSERT(y + dy < desired[x + dx].size(), "invalid y");
            //ASSERT(desired[x + dx][y + dy] == '.', "already set");
            desired[x + dx][y + dy] = image[dx][dy];
        }
    }
}

GuidanceMap::GuidanceMap(MapType type, const Map &map)
    : desired(map.get_rows(), std::string(map.get_cols(), '.')) {

    if (type == MapType::RANDOM) {
        set_random(map);
    } else if (type == MapType::CITY) {
        set_city(map);
    } else if (type == MapType::GAME) {
        set_game(map);
    } else if (type == MapType::WAREHOUSE) {
        set_warehouse(map);
    } else if (type == MapType::SORTATION) {
        set_sortation(map);
    } else {
        FAILED_ASSERT("undefined map");
    }

    for (uint32_t x = 0; x < map.get_rows(); x++) {
        for (uint32_t y = 0; y < map.get_cols(); y++) {
            if (!map.is_free(x * map.get_cols() + y + 1)) {
                ASSERT(desired[x][y] == '.' || desired[x][y] == '@', "failed to set @");
                desired[x][y] = '@';
            }
        }
    }

    /*{
        std::ofstream output("Solution/Data/guidance_map_warehouse.txt");
        output << *this;
    }
    std::exit(0);*/
}

uint32_t GuidanceMap::get_rows() const {
    return desired.size();
}

uint32_t GuidanceMap::get_cols() const {
    ASSERT(!desired.empty(), "empty");
    return desired.back().size();
}

char GuidanceMap::get(uint32_t x, uint32_t y) const {
    ASSERT(x < desired.size(), "invalid x");
    ASSERT(y < desired[x].size(), "invalid y");
    return desired[x][y];
}

std::ostream &operator<<(std::ostream &output, const GuidanceMap &map) {
    output << map.desired.size() << ' ' << map.desired.back().size() << '\n';
    for (const auto &row: map.desired) {
        output << row << '\n';
    }
    return output;
}

std::istream &operator>>(std::istream &input, GuidanceMap &map) {
    uint32_t rows, cols;
    input >> rows >> cols;
    map.desired.resize(rows);
    for (auto &row: map.desired) {
        input >> row;
        ASSERT(row.size() == cols, "invalid sizes");
    }
    return input;
}

GuidanceMap &get_guidance_map() {
    static GuidanceMap map;
    return map;
}
