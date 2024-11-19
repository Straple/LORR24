#include <Objects/assert.hpp>
#include <Objects/map.hpp>

Map::Map(const SharedEnvironment &env) : rows(env.rows), cols(env.cols) {
    ASSERT(get_size() == env.map.size(), "size mismatch");
    map.resize(env.map.size());
    for (uint32_t pos = 0; pos < map.size(); pos++) {
        map[pos] = env.map[pos] == 0;
    }
}

uint32_t Map::get_rows() const {
    return rows;
}

uint32_t Map::get_cols() const {
    return cols;
}

uint32_t Map::get_size() const {
    return rows * cols;
}

bool Map::is_free(uint32_t pos) const {
    ASSERT(pos < map.size(), "invalid pos");
    return map[pos];
}

Map &get_map() {
    static Map map;
    return map;
}
